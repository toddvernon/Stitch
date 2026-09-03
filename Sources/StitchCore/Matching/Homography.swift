import Foundation
import simd

/// 3x3 projective transform mapping homogeneous source points to target points.
public typealias Homography = simd_double3x3

public enum HomographyEstimator {

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

        // Build M = AᵀA directly (9x9), A being the 2n x 9 DLT matrix.
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

    /// Hartley normalization: centroid to origin, mean distance √2.
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
                    let app = a[p * n + p], aqq = a[q * n + q]
                    let tau = (aqq - app) / (2 * apq)
                    let t = (tau >= 0 ? 1.0 : -1.0) / (abs(tau) + sqrt(1 + tau * tau))
                    let c = 1 / sqrt(1 + t * t)
                    let s = t * c
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
