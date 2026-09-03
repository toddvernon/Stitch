import Foundation
import simd

public struct MeshRefinerResult {
    public var meshes: [Int: WarpMesh]
    /// RMS pairwise alignment residual of inlier matches before/after, px.
    public var initialRMS: Double
    public var finalRMS: Double
}

/// Milestone-5 parallax stage: per-image warp meshes that close the residual
/// misalignment the rotational camera model cannot represent (content-
/// preserving warps in the spirit of Zhang & Liu CVPR 2014 / APAP).
///
/// For every inlier match between images i and j the corrected positions must
/// agree through the bundle-adjusted homography: W_i(u_i) = H_ij(W_j(u_j)).
/// Offsets are solved by alternating regularized linear least squares per
/// image (data term above; graph-Laplacian smoothness so each region deforms
/// near-rigidly; a small zero prior pinning unconstrained vertices).
public enum MeshRefiner {

    /// Matches with residuals beyond this are treated as mismatches and skipped.
    static let residualCutoff = 40.0

    public static func refine(group: PanoramaGroup,
                              features: [[Feature]],
                              cameras: [Int: Camera],
                              vertexSpacing: Int = 100,
                              sweeps: Int = 8,
                              smoothness: Double = 1.0,
                              rigidity: Double = 0.02) -> MeshRefinerResult {
        // One constraint per match direction: content at `point` in `image`
        // must land where `otherPoint` in `otherImage` projects through `h`
        // (centered-coordinate homography other→image).
        struct Constraint {
            var image: Int
            var point: SIMD2<Double>
            var otherImage: Int
            var otherPoint: SIMD2<Double>
            var h: simd_double3x3
        }

        var meshes: [Int: WarpMesh] = [:]
        for (idx, cam) in cameras {
            meshes[idx] = WarpMesh(width: cam.width, height: cam.height, vertexSpacing: vertexSpacing)
        }

        func predicted(_ c: Constraint) -> SIMD2<Double>? {
            let other = cameras[c.otherImage]!
            let cam = cameras[c.image]!
            let corrected = c.otherPoint + meshes[c.otherImage]!.offset(x: c.otherPoint.x, y: c.otherPoint.y)
            let centered = other.centered(corrected)
            let w = c.h * SIMD3(centered.x, centered.y, 1)
            guard w.z > 1e-9 else { return nil }
            return SIMD2(w.x / w.z + Double(cam.width) / 2, w.y / w.z + Double(cam.height) / 2)
        }

        var constraints: [Constraint] = []
        var byImage: [Int: [Int]] = [:]  // image index → constraint indices
        for pair in group.pairs {
            guard let camA = cameras[pair.indexA], let camB = cameras[pair.indexB] else { continue }
            let hBtoA = camA.homography(from: camB)
            let hAtoB = camB.homography(from: camA)
            for k in pair.geometry.inlierIndices {
                let m = pair.matches[k]
                let fa = features[pair.indexA][m.indexA]
                let fb = features[pair.indexB][m.indexB]
                let ua = SIMD2(Double(fa.x), Double(fa.y))
                let ub = SIMD2(Double(fb.x), Double(fb.y))
                let forward = Constraint(image: pair.indexA, point: ua,
                                         otherImage: pair.indexB, otherPoint: ub, h: hBtoA)
                let backward = Constraint(image: pair.indexB, point: ub,
                                          otherImage: pair.indexA, otherPoint: ua, h: hAtoB)
                for c in [forward, backward] {
                    guard let p = predicted(c), length(p - c.point) < residualCutoff else { continue }
                    byImage[c.image, default: []].append(constraints.count)
                    constraints.append(c)
                }
            }
        }

        func rms() -> Double {
            var sum = 0.0
            var count = 0
            for c in constraints {
                guard let p = predicted(c) else { continue }
                let corrected = c.point + meshes[c.image]!.offset(x: c.point.x, y: c.point.y)
                sum += length_squared(p - corrected)
                count += 1
            }
            return count > 0 ? sqrt(sum / Double(count)) : 0
        }

        let initialRMS = rms()

        for _ in 0..<sweeps {
            for image in cameras.keys.sorted() {
                guard let indices = byImage[image], !indices.isEmpty else { continue }
                var mesh = meshes[image]!
                let nV = mesh.cols * mesh.rows

                var a = [Double](repeating: 0, count: nV * nV)
                var bx = [Double](repeating: 0, count: nV)
                var by = [Double](repeating: 0, count: nV)

                // Data term: Σφ_k v_k = predicted − point, per axis.
                for ci in indices {
                    let c = constraints[ci]
                    guard let p = predicted(c) else { continue }
                    let t = p - c.point
                    let stencil = mesh.stencil(x: c.point.x, y: c.point.y)
                    for (i, wi) in stencil {
                        bx[i] += wi * t.x
                        by[i] += wi * t.y
                        for (j, wj) in stencil {
                            a[i * nV + j] += wi * wj
                        }
                    }
                }

                // Smoothness: penalize differences along grid edges.
                for r in 0..<mesh.rows {
                    for col in 0..<mesh.cols {
                        let i = r * mesh.cols + col
                        var neighbors: [Int] = []
                        if col + 1 < mesh.cols { neighbors.append(i + 1) }
                        if r + 1 < mesh.rows { neighbors.append(i + mesh.cols) }
                        for j in neighbors {
                            a[i * nV + i] += smoothness
                            a[j * nV + j] += smoothness
                            a[i * nV + j] -= smoothness
                            a[j * nV + i] -= smoothness
                        }
                        a[i * nV + i] += rigidity
                    }
                }

                if let vx = BundleAdjuster.choleskySolve(a, bx, n: nV),
                   let vy = BundleAdjuster.choleskySolve(a, by, n: nV) {
                    mesh.dx = vx
                    mesh.dy = vy
                    meshes[image] = mesh
                }
            }
        }

        return MeshRefinerResult(meshes: meshes, initialRMS: initialRMS, finalRMS: rms())
    }
}
