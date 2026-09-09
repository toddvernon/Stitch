import Foundation
import simd

// The pair model for strip mode (DESIGN.md, "stitch strip", step 2): a
// 4-DOF similarity on the facade plane. PairEstimator fits it from two-point
// RANSAC samples, and StripAligner solves one per image globally, which is
// only a linear problem because of the parameterization chosen here.
// `Homography` is the corresponding model for rotational panoramas.

/// 2-D similarity transform (uniform scale, rotation, translation), stored as
/// the complex scale (a + ib) and translation: x' = a·x − b·y + tx,
/// y' = b·x + a·y + ty. Linear in its four parameters, which is what makes
/// the strip aligner's global solve a plain least-squares problem.
public struct Similarity: Equatable {
    /// Real part of the complex scale: scale · cos(rotation).
    public var a: Double
    /// Imaginary part of the complex scale: scale · sin(rotation).
    public var b: Double
    /// Translation, applied after the scale-rotation.
    public var tx: Double
    public var ty: Double

    /// Maps every point to itself.
    public static let identity = Similarity(a: 1, b: 0, tx: 0, ty: 0)

    public init(a: Double, b: Double, tx: Double, ty: Double) {
        self.a = a
        self.b = b
        self.tx = tx
        self.ty = ty
    }

    /// Polar form. `rotation` is in radians, positive from +x toward +y
    /// (clockwise on screen, since image y points down).
    public init(scale: Double, rotation: Double, translation: SIMD2<Double>) {
        a = scale * cos(rotation)
        b = scale * sin(rotation)
        tx = translation.x
        ty = translation.y
    }

    /// Uniform scale factor, |a + ib|.
    public var scale: Double { hypot(a, b) }
    /// Rotation in radians, arg(a + ib).
    public var rotation: Double { atan2(b, a) }
    public var translation: SIMD2<Double> { SIMD2(tx, ty) }

    /// Transforms one point.
    @inline(__always)
    public func apply(_ p: SIMD2<Double>) -> SIMD2<Double> {
        SIMD2(a * p.x - b * p.y + tx, b * p.x + a * p.y + ty)
    }

    /// Exact inverse: (a + ib)⁻¹ = conj(a + ib) / |a + ib|², and the
    /// translation is pulled back through that.
    public var inverse: Similarity {
        let d = a * a + b * b
        let ia = a / d, ib = -b / d
        return Similarity(a: ia, b: ib,
                          tx: -(ia * tx - ib * ty),
                          ty: -(ib * tx + ia * ty))
    }

    /// `self ∘ other`: applies `other` first, then `self`.
    public func composed(with other: Similarity) -> Similarity {
        Similarity(a: a * other.a - b * other.b,
                   b: b * other.a + a * other.b,
                   tx: a * other.tx - b * other.ty + tx,
                   ty: b * other.tx + a * other.ty + ty)
    }

    /// The same map as a 3×3, for the code paths shared with the rotational
    /// model (RANSAC inlier tests and `PairGeometry` store a `Homography`).
    public var homography: Homography {
        Homography(rows: [SIMD3(a, -b, tx), SIMD3(b, a, ty), SIMD3(0, 0, 1)])
    }

    /// Reads a similarity back out of a homography with an affine last row.
    /// Returns nil for a genuinely projective matrix, so a `PairGeometry`
    /// estimated with the homography model reports no similarity. Note that
    /// simd matrices are column-major: h[c][r] is column c, row r.
    public init?(homography h: Homography) {
        let r0 = SIMD3(h[0][0], h[1][0], h[2][0])
        let r1 = SIMD3(h[0][1], h[1][1], h[2][1])
        let r2 = SIMD3(h[0][2], h[1][2], h[2][2])
        guard abs(r2.x) < 1e-9, abs(r2.y) < 1e-9, abs(r2.z) > 1e-12 else { return nil }
        let s = 1 / r2.z
        self.init(a: r0.x * s, b: r1.x * s, tx: r0.z * s, ty: r1.z * s)
    }

    /// Least-squares fit mapping `from` to `to` (≥ 2 points; the two-point
    /// case is exact). Optional per-point weights for IRLS.
    public static func fit(from: [SIMD2<Double>], to: [SIMD2<Double>],
                           weights: [Double]? = nil) -> Similarity? {
        precondition(from.count == to.count)
        guard from.count >= 2 else { return nil }
        // Center both point sets on their (weighted) means first: the
        // scale-rotation then decouples from the translation, and the
        // translation falls out of the means at the end.
        var wSum = 0.0
        var pMean = SIMD2<Double>(0, 0), qMean = SIMD2<Double>(0, 0)
        for k in from.indices {
            let w = weights?[k] ?? 1
            wSum += w
            pMean += w * from[k]
            qMean += w * to[k]
        }
        guard wSum > 0 else { return nil }
        pMean /= wSum
        qMean /= wSum
        // Complex least squares: (a + ib) = Σ w·conj(p̃)·q̃ / Σ w·|p̃|².
        var num = SIMD2<Double>(0, 0)
        var den = 0.0
        for k in from.indices {
            let w = weights?[k] ?? 1
            let p = from[k] - pMean, q = to[k] - qMean
            num.x += w * (p.x * q.x + p.y * q.y)
            num.y += w * (p.x * q.y - p.y * q.x)
            den += w * (p.x * p.x + p.y * p.y)
        }
        // den ≈ 0: all source points coincide, no scale is determined.
        guard den > 1e-12 else { return nil }
        let a = num.x / den, b = num.y / den
        let t = qMean - SIMD2(a * pMean.x - b * pMean.y, b * pMean.x + a * pMean.y)
        return Similarity(a: a, b: b, tx: t.x, ty: t.y)
    }
}
