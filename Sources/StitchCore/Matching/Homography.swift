import Foundation
import simd

// The pair model for rotational panoramas: a 3×3 projective transform, the
// normalized DLT estimator that PairEstimator's four-point RANSAC samples
// call, and the small symmetric eigensolver both it and the straightening
// step in PanoramaAligner rely on.

/// 3x3 projective transform mapping homogeneous source points to target points.
/// simd matrices are column-major, so `h[c][r]` is column c, row r.
public typealias Homography = simd_double3x3

/// Direct linear transform estimation of a homography from point pairs
/// (Hartley & Zisserman, Multiple View Geometry, §4.1; normalization §4.4).
public enum HomographyEstimator {

    /// Applies `h` to a Euclidean point and dehomogenizes. No guard on
    /// w = 0; callers only feed it points near the images.
    @inline(__always)
    public static func project(_ h: Homography, _ p: SIMD2<Double>) -> SIMD2<Double> {
        let v = h * SIMD3<Double>(p.x, p.y, 1)
        return SIMD2(v.x / v.z, v.y / v.z)
    }

    /// Normalized DLT (Hartley) from ≥4 correspondences. Returns the H that
    /// maps `from` points to `to` points, or nil if degenerate.
    public static func dlt(from: [SIMD2<Double>], to: [SIMD2<Double>]) -> Homography? {
        precondition(from.count == to.count)
        guard from.count >= 4 else { return nil }

        guard let (nFrom, tFrom) = normalize(from),
              let (nTo, tTo) = normalize(to) else { return nil }

        // Build M = AᵀA directly (9x9), A being the 2n x 9 DLT matrix. The
        // smallest eigenvector of AᵀA is the smallest right singular vector
        // of A, and accumulating the 9x9 keeps the solve fixed-size however
        // many correspondences come in (4 per RANSAC sample, all inliers on
        // the refit).
        var m = [Double](repeating: 0, count: 81)
        for k in 0..<nFrom.count {
            let x = nFrom[k].x, y = nFrom[k].y
            let u = nTo[k].x, v = nTo[k].y
            let rows: [[Double]] = [
                [-x, -y, -1, 0, 0, 0, u * x, u * y, u],
                [0, 0, 0, -x, -y, -1, v * x, v * y, v],
            ]
            for row in rows {
                for i in 0..<9 {
                    for j in i..<9 {
                        m[i * 9 + j] += row[i] * row[j]
                    }
                }
            }
        }
        for i in 0..<9 {
            for j in 0..<i {
                m[i * 9 + j] = m[j * 9 + i]
            }
        }

        guard let h = smallestEigenvector(m, n: 9) else { return nil }
        let hNorm = Homography(rows: [
            SIMD3(h[0], h[1], h[2]),
            SIMD3(h[3], h[4], h[5]),
            SIMD3(h[6], h[7], h[8]),
        ])
        // A singular H (three collinear sample points, say) is no model at all.
        guard abs(hNorm.determinant) > 1e-12 else { return nil }

        // Denormalize: H = Tto⁻¹ · Ĥ · Tfrom
        let result = tTo.inverse * hNorm * tFrom
        // Scale so h33 = 1 when possible (cosmetic, aids comparison).
        let s = result[2][2]  // column-major: [2][2] is row 2, col 2 either way
        if abs(s) > 1e-12 {
            return result * (1 / s)
        }
        return result
    }

    /// Hartley normalization: centroid to origin, mean distance √2. Without
    /// it the DLT is badly conditioned: with pixel coordinates in the
    /// thousands the u·x entries of A are 10⁶ larger than the constant ones.
    private static func normalize(_ pts: [SIMD2<Double>]) -> ([SIMD2<Double>], Homography)? {
        let n = Double(pts.count)
        var centroid = SIMD2<Double>(0, 0)
        for p in pts { centroid += p }
        centroid /= n
        var meanDist = 0.0
        for p in pts { meanDist += length(p - centroid) }
        meanDist /= n
        guard meanDist > 1e-12 else { return nil }
        let s = sqrt(2) / meanDist
        let t = Homography(rows: [
            SIMD3(s, 0, -s * centroid.x),
            SIMD3(0, s, -s * centroid.y),
            SIMD3(0, 0, 1),
        ])
        return (pts.map { SIMD2(s * ($0.x - centroid.x), s * ($0.y - centroid.y)) }, t)
    }

    /// Eigenvector of the smallest eigenvalue of a symmetric n x n matrix,
    /// via cyclic Jacobi rotations. Small n only (we use n = 9).
    static func smallestEigenvector(_ matrix: [Double], n: Int) -> [Double]? {
        var a = matrix
        var v = [Double](repeating: 0, count: n * n)
        for i in 0..<n { v[i * n + i] = 1 }

        // 50 sweeps is far more than n = 9 ever needs; the off-diagonal
        // norm check below exits as soon as A is diagonal to round-off.
        for _ in 0..<50 {
            var off = 0.0
            for p in 0..<n {
                for q in (p + 1)..<n {
                    off += a[p * n + q] * a[p * n + q]
                }
            }
            if off < 1e-22 { break }

            for p in 0..<n {
                for q in (p + 1)..<n {
                    let apq = a[p * n + q]
                    if abs(apq) < 1e-30 { continue }
                    // Rotation angle that zeroes a[p][q]: t = tan θ is the
                    // smaller-magnitude root of t² + 2τt − 1 = 0, the
                    // numerically stable choice (Numerical Recipes §11.1).
                    let app = a[p * n + p], aqq = a[q * n + q]
                    let tau = (aqq - app) / (2 * apq)
                    let t = (tau >= 0 ? 1.0 : -1.0) / (abs(tau) + sqrt(1 + tau * tau))
                    let c = 1 / sqrt(1 + t * t)
                    let s = t * c
                    // A ← Jᵀ A J applied as columns then rows; V accumulates J.
                    for k in 0..<n {
                        let akp = a[k * n + p], akq = a[k * n + q]
                        a[k * n + p] = c * akp - s * akq
                        a[k * n + q] = s * akp + c * akq
                    }
                    for k in 0..<n {
                        let apk = a[p * n + k], aqk = a[q * n + k]
                        a[p * n + k] = c * apk - s * aqk
                        a[q * n + k] = s * apk + c * aqk
                    }
                    for k in 0..<n {
                        let vkp = v[k * n + p], vkq = v[k * n + q]
                        v[k * n + p] = c * vkp - s * vkq
                        v[k * n + q] = s * vkp + c * vkq
                    }
                }
            }
        }

        // The diagonal of A now holds the eigenvalues, the columns of V the
        // eigenvectors; pick the column of the smallest eigenvalue.
        var minIdx = 0
        var minVal = a[0]
        for i in 1..<n {
            let val = a[i * n + i]
            if val < minVal {
                minVal = val
                minIdx = i
            }
        }
        var eigvec = [Double](repeating: 0, count: n)
        for k in 0..<n { eigvec[k] = v[k * n + minIdx] }
        return eigvec
    }
}
