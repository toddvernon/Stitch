import Foundation
import simd

/// Cameras for one panorama group, keyed by image index in the original set.
public struct AlignmentResult {
    public var cameras: [Int: Camera]
    public var finalRMS: Double
}

/// Incremental alignment of a recognized panorama (Brown & Lowe IJCV 2007 §4):
/// seed with the best-connected pair, add images best-first, initializing each
/// from the homography to its best placed neighbor, bundle adjusting as we go
/// (L2 during growth, Huber σ=2px for the final polish), then straighten.
public enum PanoramaAligner {

    public static func align(group: PanoramaGroup,
                             features: [[Feature]],
                             imageSizes: [(width: Int, height: Int)],
                             focalHints: [Int: Double]) -> AlignmentResult? {
        guard group.imageIndices.count >= 2 else { return nil }

        func defaultFocal(_ idx: Int) -> Double {
            focalHints[idx] ?? 0.9 * Double(max(imageSizes[idx].width, imageSizes[idx].height))
        }
        func makeCamera(_ idx: Int) -> Camera {
            Camera(focal: defaultFocal(idx), width: imageSizes[idx].width, height: imageSizes[idx].height)
        }

        // Inlier counts between image pairs, for placement order.
        var inlierCount: [Int: Int] = [:]  // key: min*N+max over image indices
        let n = features.count
        for p in group.pairs {
            inlierCount[min(p.indexA, p.indexB) * n + max(p.indexA, p.indexB)] = p.geometry.inlierIndices.count
        }

        guard let seedPair = group.pairs.max(by: { $0.geometry.inlierIndices.count < $1.geometry.inlierIndices.count })
        else { return nil }

        var cameras: [Int: Camera] = [:]
        cameras[seedPair.indexA] = makeCamera(seedPair.indexA)
        var camB = makeCamera(seedPair.indexB)
        initializeRotation(of: &camB, from: cameras[seedPair.indexA]!, pair: seedPair, placedIsA: true)
        cameras[seedPair.indexB] = camB

        var remaining = Set(group.imageIndices).subtracting(cameras.keys)
        runBA(cameras: &cameras, group: group, features: features, huberSigma: nil)

        while !remaining.isEmpty {
            // Pick the unplaced image with the most inliers to any placed image.
            var bestImage: Int?
            var bestPair: VerifiedPair?
            var bestCount = -1
            for p in group.pairs {
                let aPlaced = cameras[p.indexA] != nil
                let bPlaced = cameras[p.indexB] != nil
                guard aPlaced != bPlaced else { continue }
                let candidate = aPlaced ? p.indexB : p.indexA
                let count = p.geometry.inlierIndices.count
                if count > bestCount {
                    bestCount = count
                    bestImage = candidate
                    bestPair = p
                }
            }
            guard let image = bestImage, let pair = bestPair else { break }

            var cam = makeCamera(image)
            let placedIsA = cameras[pair.indexA] != nil
            let placed = cameras[placedIsA ? pair.indexA : pair.indexB]!
            initializeRotation(of: &cam, from: placed, pair: pair, placedIsA: placedIsA)
            cameras[image] = cam
            remaining.remove(image)

            runBA(cameras: &cameras, group: group, features: features, huberSigma: nil)
        }

        let finalRMS = runBA(cameras: &cameras, group: group, features: features,
                             huberSigma: 2.0, maxIterations: 60)
        straighten(cameras: &cameras)
        return AlignmentResult(cameras: cameras, finalRMS: finalRMS)
    }

    /// Initialize an unplaced camera's rotation from a verified pair with a
    /// placed one: R_B = (K_B⁻¹ · H_c · K_A · R_Aᵀ)ᵀ-style decomposition of
    /// the pair homography (paper eq. 1), projected back onto SO(3).
    private static func initializeRotation(of camera: inout Camera, from placed: Camera,
                                           pair: VerifiedPair, placedIsA: Bool) {
        // pair.geometry.homography maps A pixels (top-left origin) → B pixels.
        // Convert to centered coordinates on both sides.
        let (wa, ha) = placedIsA ? (placed.width, placed.height) : (camera.width, camera.height)
        let (wb, hb) = placedIsA ? (camera.width, camera.height) : (placed.width, placed.height)
        let tA = translation(Double(wa) / 2, Double(ha) / 2)
        let tB = translation(-Double(wb) / 2, -Double(hb) / 2)
        let hCentered = tB * pair.geometry.homography * tA

        if placedIsA {
            // H_c ≈ K_B R_B R_Aᵀ K_A⁻¹  →  R_B = normalize(K_B⁻¹ H_c K_A) R_A
            var m = camera.intrinsicsInverse * hCentered * placed.intrinsics
            m = normalizeScale(m)
            camera.rotation = SO3.orthonormalize(m * placed.rotation)
        } else {
            // Placed is B: R_A = (normalize(K_B⁻¹ H_c K_A))ᵀ-composed inverse.
            var m = placed.intrinsicsInverse * hCentered * camera.intrinsics
            m = normalizeScale(m)
            camera.rotation = SO3.orthonormalize(m.inverse * placed.rotation)
        }
    }

    private static func translation(_ x: Double, _ y: Double) -> simd_double3x3 {
        simd_double3x3(rows: [SIMD3(1, 0, x), SIMD3(0, 1, y), SIMD3(0, 0, 1)])
    }

    private static func normalizeScale(_ m: simd_double3x3) -> simd_double3x3 {
        let det = m.determinant
        guard abs(det) > 1e-12 else { return m }
        let s = det > 0 ? pow(det, 1.0 / 3.0) : -pow(-det, 1.0 / 3.0)
        return m * (1 / s)
    }

    /// Bundle adjust all currently placed cameras using inlier matches of all
    /// pairs whose two images are both placed.
    @discardableResult
    private static func runBA(cameras: inout [Int: Camera],
                              group: PanoramaGroup,
                              features: [[Feature]],
                              huberSigma: Double?,
                              maxIterations: Int = 20) -> Double {
        let placed = cameras.keys.sorted()
        guard placed.count >= 2 else { return .infinity }
        let localIndex = Dictionary(uniqueKeysWithValues: placed.enumerated().map { ($1, $0) })
        var cams = placed.map { cameras[$0]! }

        var observations: [MatchObservation] = []
        for p in group.pairs {
            guard let la = localIndex[p.indexA], let lb = localIndex[p.indexB] else { continue }
            let ca = cams[la], cb = cams[lb]
            for k in p.geometry.inlierIndices {
                let m = p.matches[k]
                let fa = features[p.indexA][m.indexA]
                let fb = features[p.indexB][m.indexB]
                let pa = ca.centered(SIMD2(Double(fa.x), Double(fa.y)))
                let pb = cb.centered(SIMD2(Double(fb.x), Double(fb.y)))
                observations.append(MatchObservation(cameraA: la, cameraB: lb, pointA: pa, pointB: pb))
                observations.append(MatchObservation(cameraA: lb, cameraB: la, pointA: pb, pointB: pa))
            }
        }

        let rms = BundleAdjuster.adjust(cameras: &cams, observations: observations,
                                        huberSigma: huberSigma, maxIterations: maxIterations)
        for (imageIdx, local) in localIndex {
            cameras[imageIdx] = cams[local]
        }
        return rms
    }

    /// Automatic straightening (paper §5): the up-vector is the null eigenvector
    /// of the covariance of camera X-axes. Rotates the world so it is vertical
    /// and the mean viewing direction is forward.
    static func straighten(cameras: inout [Int: Camera]) {
        guard cameras.count >= 2 else { return }
        var cov = [Double](repeating: 0, count: 9)
        var meanUp = SIMD3<Double>.zero
        var meanForward = SIMD3<Double>.zero
        for cam in cameras.values {
            let rT = cam.rotation.transpose
            let x = rT.columns.0          // camera X axis in world
            let up = -rT.columns.1        // image y points down
            let forward = rT.columns.2
            for i in 0..<3 {
                for j in 0..<3 {
                    cov[i * 3 + j] += x[i] * x[j]
                }
            }
            meanUp += up
            meanForward += forward
        }
        guard let u = HomographyEstimator.smallestEigenvector(cov, n: 3) else { return }
        var up = SIMD3(u[0], u[1], u[2])
        if dot(up, meanUp) < 0 { up = -up }
        up = normalize(up)

        var forward = meanForward - dot(meanForward, up) * up
        guard length(forward) > 1e-9 else { return }
        forward = normalize(forward)
        let yNew = -up
        let xNew = cross(yNew, forward)
        // World-from-straightened basis; R'_i = R_i · W keeps projections identical
        // while expressing rays in the straightened frame.
        let w = simd_double3x3(xNew, yNew, forward)
        for key in cameras.keys {
            cameras[key]!.rotation = cameras[key]!.rotation * w
        }
    }
}
