import Foundation
import simd

// Stages 5 and 6 of the pipeline (DESIGN.md): the incremental placement
// schedule that drives BundleAdjuster, and automatic straightening. Takes a
// recognized PanoramaGroup and returns a Camera per image for MeshRefiner
// and the compositor.

/// Cameras for one panorama group, keyed by image index in the original set.
public struct AlignmentResult {
    public var cameras: [Int: Camera]
    /// RMS reprojection error of the final robust pass, px at registration
    /// scale.
    public var finalRMS: Double
}

/// Incremental alignment of a recognized panorama (Brown & Lowe IJCV 2007 §4):
/// seed with the best-connected pair, add images best-first, initializing each
/// from the homography to its best placed neighbor, bundle adjusting as we go
/// (L2 during growth, Huber σ=2px for the final polish), then straighten.
public enum PanoramaAligner {

    /// Aligns one group. `focalHints` are EXIF-derived focal lengths in
    /// registration-scale pixels, keyed by image index; an image without one
    /// starts at 0.9 × its long side (about a 32 mm lens on full frame),
    /// which bundle adjustment corrects within a few iterations. Returns nil
    /// for groups under two images or with no verified pair.
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

        // Inlier counts between image pairs, for placement order. (Currently
        // unused: the placement loop below reads the pair list directly.)
        var inlierCount: [Int: Int] = [:]  // key: min*N+max over image indices
        let n = features.count
        for p in group.pairs {
            inlierCount[min(p.indexA, p.indexB) * n + max(p.indexA, p.indexB)] = p.geometry.inlierIndices.count
        }

        // Seed with the pair holding the most inliers (paper §4: the best
        // matching pair first), the most trustworthy homography to decompose.
        guard let seedPair = group.pairs.max(by: { $0.geometry.inlierIndices.count < $1.geometry.inlierIndices.count })
        else { return nil }

        var cameras: [Int: Camera] = [:]
        cameras[seedPair.indexA] = makeCamera(seedPair.indexA)
        var camB = makeCamera(seedPair.indexB)
        initializeRotation(of: &camB, from: cameras[seedPair.indexA]!, pair: seedPair, placedIsA: true)
        cameras[seedPair.indexB] = camB

        // Bundle adjust after every addition (the paper's schedule), so each
        // new image is initialized against cameras that already agree with
        // each other rather than against a single unrefined neighbor.
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
            // No pair bridges placed and unplaced: the group is not connected
            // (shouldn't happen for a connected component). Stop rather than
            // guess.
            guard let image = bestImage, let pair = bestPair else { break }

            var cam = makeCamera(image)
            let placedIsA = cameras[pair.indexA] != nil
            let placed = cameras[placedIsA ? pair.indexA : pair.indexB]!
            initializeRotation(of: &cam, from: placed, pair: pair, placedIsA: placedIsA)
            cameras[image] = cam
            remaining.remove(image)

            runBA(cameras: &cameras, group: group, features: features, huberSigma: nil)
        }

        // Final polish with the robust loss and a longer iteration budget,
        // then straighten.
        let finalRMS = runBA(cameras: &cameras, group: group, features: features,
                             huberSigma: 2.0, maxIterations: 60)
        straighten(cameras: &cameras)
        return AlignmentResult(cameras: cameras, finalRMS: finalRMS)
    }

    /// Initialize an unplaced camera's rotation from its verified pair with
    /// a placed one, by decomposing the pair homography H = K_B R_B R_Aᵀ K_A⁻¹
    /// (paper eq. 1): R_B R_Aᵀ ≈ K_B⁻¹ H K_A up to scale, projected back onto
    /// SO(3). The intrinsics are whatever the cameras currently hold, so the
    /// result is approximate; the bundle adjustment that follows fixes it.
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

    /// Pure-translation homography, for moving between top-left and centered
    /// pixel origins.
    private static func translation(_ x: Double, _ y: Double) -> simd_double3x3 {
        simd_double3x3(rows: [SIMD3(1, 0, x), SIMD3(0, 1, y), SIMD3(0, 0, 1)])
    }

    /// Divides out a homography's arbitrary projective scale using
    /// det(sR) = s³, so what reaches `orthonormalize` is close to a rotation
    /// rather than a multiple of one (and has positive determinant).
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
                // Both directions, so the error is symmetric in the pair
                // (paper eq. 8 sums over the residuals in both images).
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
        // The X axes of a hand-held pan all lie roughly in the horizontal
        // plane; the direction they span least is the vertical.
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
        // An eigenvector has no sign; orient it along the cameras' mean up so
        // the panorama doesn't come out upside down.
        var up = SIMD3(u[0], u[1], u[2])
        if dot(up, meanUp) < 0 { up = -up }
        up = normalize(up)

        // Forward = mean viewing direction made perpendicular to up, so the
        // straightened world has the panorama centered dead ahead.
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
