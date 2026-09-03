import Foundation
import simd

/// Rotational camera model (Brown & Lowe IJCV 2007 §2): a rotation and a focal
/// length. Pixel coordinates are centered (origin at the principal point,
/// x right, y down); `Camera` maps world ray directions to centered pixels via
/// p̃ = K·R·d, with K = diag(f, f, 1).
public struct Camera {
    /// World-to-camera rotation: p_cam = rotation · d_world.
    public var rotation: simd_double3x3
    /// Focal length in pixels (at registration scale).
    public var focal: Double
    public var width: Int
    public var height: Int

    public init(rotation: simd_double3x3 = matrix_identity_double3x3,
                focal: Double, width: Int, height: Int) {
        self.rotation = rotation
        self.focal = focal
        self.width = width
        self.height = height
    }

    public var intrinsics: simd_double3x3 {
        simd_double3x3(diagonal: SIMD3(focal, focal, 1))
    }

    public var intrinsicsInverse: simd_double3x3 {
        simd_double3x3(diagonal: SIMD3(1 / focal, 1 / focal, 1))
    }

    /// Centered pixel for a world direction, or nil if it points behind the camera.
    public func project(_ dWorld: SIMD3<Double>) -> SIMD2<Double>? {
        let p = rotation * dWorld
        guard p.z > 1e-9 else { return nil }
        return SIMD2(focal * p.x / p.z, focal * p.y / p.z)
    }

    /// World ray direction (unit) for a centered pixel.
    public func ray(_ pixel: SIMD2<Double>) -> SIMD3<Double> {
        let d = rotation.transpose * SIMD3(pixel.x / focal, pixel.y / focal, 1)
        return normalize(d)
    }

    /// Centers a top-left-origin pixel coordinate.
    public func centered(_ p: SIMD2<Double>) -> SIMD2<Double> {
        SIMD2(p.x - Double(width) / 2, p.y - Double(height) / 2)
    }

    /// Homography mapping centered pixels of `other` into this camera's
    /// centered pixels: H = K_i R_i R_j^T K_j^{-1} (paper eq. 1).
    public func homography(from other: Camera) -> simd_double3x3 {
        intrinsics * rotation * other.rotation.transpose * other.intrinsicsInverse
    }
}

public enum SO3 {
    /// so(3) generators: d/dθ_k of exp([θ]×) at θ = 0.
    public static let generators: [simd_double3x3] = [
        simd_double3x3(rows: [SIMD3(0, 0, 0), SIMD3(0, 0, -1), SIMD3(0, 1, 0)]),
        simd_double3x3(rows: [SIMD3(0, 0, 1), SIMD3(0, 0, 0), SIMD3(-1, 0, 0)]),
        simd_double3x3(rows: [SIMD3(0, -1, 0), SIMD3(1, 0, 0), SIMD3(0, 0, 0)]),
    ]

    /// Rodrigues exponential map.
    public static func exp(_ w: SIMD3<Double>) -> simd_double3x3 {
        let theta = length(w)
        if theta < 1e-12 { return matrix_identity_double3x3 }
        let axis = w / theta
        let k = simd_double3x3(rows: [
            SIMD3(0, -axis.z, axis.y),
            SIMD3(axis.z, 0, -axis.x),
            SIMD3(-axis.y, axis.x, 0),
        ])
        return matrix_identity_double3x3 + sin(theta) * k + (1 - cos(theta)) * (k * k)
    }

    /// Projects an approximate rotation back onto SO(3) (nearest orthonormal
    /// matrix, det +1) via Gram-Schmidt on the columns.
    public static func orthonormalize(_ m: simd_double3x3) -> simd_double3x3 {
        var x = normalize(m.columns.0)
        var y = m.columns.1 - dot(m.columns.1, x) * x
        y = normalize(y)
        let z = cross(x, y)
        if simd_double3x3(x, y, z).determinant < 0 { x = -x }
        return simd_double3x3(x, y, cross(x, y))
    }
}
