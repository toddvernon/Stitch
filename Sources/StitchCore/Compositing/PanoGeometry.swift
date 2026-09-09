// The rotational output space. PanoGeometry is the single place pano pixels
// map to world ray directions; PanoProjection is the pluggable part of that
// mapping; ImageLayer is what a projected image looks like to the rest of
// compositing; LayerProjector does the projection for one camera.

import Foundation
import simd

/// How pano coordinates relate to viewing angles (see DESIGN.md, "Output
/// projections"). All keep verticals straight; they differ in what else
/// survives a wide span.
public enum PanoProjection: String, CaseIterable, Sendable {
    /// Equirectangular: u = θ, v = φ. Correct at any span.
    case spherical
    /// u = θ, v = tanφ: the horizon straightens, vertical extent stretches.
    case cylindrical
    /// Sharpless/Postle/German 2010, d = 1: verticals and radial lines
    /// straight; natural-looking architecture out to ~150°.
    case pannini

    /// Pannini's one parameter: the distance of the projection center behind
    /// the cylinder. d = 0 is rectilinear, d → ∞ is cylindrical; 1 is the
    /// paper's recommended middle and what most panorama tools ship.
    static let panniniD = 1.0

    /// (θ, φ) → projection coordinates (u, v).
    func forward(theta: Double, phi: Double) -> SIMD2<Double> {
        switch self {
        case .spherical:
            return SIMD2(theta, phi)
        case .cylindrical:
            // φ clamped to ±1.4 rad (about ±80°): tan diverges at the poles,
            // and no real camera in a horizontal sweep looks that far up.
            return SIMD2(theta, tan(min(max(phi, -1.4), 1.4)))
        case .pannini:
            let d = Self.panniniD
            // d + cosθ reaches 0 at θ = 180° for d = 1; the floor keeps u
            // finite for stray border samples well past the usable span.
            let den = max(d + cos(theta), 0.2)
            return SIMD2((d + 1) * sin(theta) / den,
                         (d + 1) * tan(min(max(phi, -1.4), 1.4)) / den)
        }
    }

    /// (u, v) → (θ, φ).
    func inverse(u: Double, v: Double) -> (theta: Double, phi: Double) {
        switch self {
        case .spherical:
            return (u, v)
        case .cylindrical:
            return (u, atan(v))
        case .pannini:
            // Solve (d+1)·sinθ − u·cosθ = u·d  via  a·sinθ + b·cosθ = R·sin(θ+ψ).
            // Then φ follows directly from v once θ is known.
            let d = Self.panniniD
            let s = d + 1
            let r = sqrt(s * s + u * u)
            let psi = atan2(-u, s)
            let theta = asin(min(max(u * d / r, -1), 1)) - psi
            let phi = atan(v * (d + cos(theta)) / (d + 1))
            return (theta, phi)
        }
    }
}

/// The shared output space: the panorama's angular extent, its projection,
/// and the mapping between pano pixels and world ray directions.
public struct PanoGeometry {
    public let projection: PanoProjection
    /// Angular extent (for reporting; pixel mapping uses u/v below).
    public let thetaMin: Double
    public let thetaMax: Double
    public let phiMin: Double
    public let phiMax: Double
    /// Projection-coordinate extent.
    let uMin: Double
    let uMax: Double
    let vMin: Double
    let vMax: Double
    /// Pixels per projection unit (≈ pixels per radian at the pano center).
    public let scale: Double
    /// Output size in pixels; height follows from the (u, v) aspect.
    public let width: Int
    public let height: Int

    /// Horizontal span in projection units (u), for natural-width sizing.
    public var uSpan: Double { uMax - uMin }

    /// Bounds the panorama by projecting a ring of border samples from every
    /// camera (image edges curve after projection, so corners alone would
    /// undershoot). Both the angular and the (u, v) extents are kept: pixels
    /// map through (u, v), the angles are for span reporting and the auto
    /// projection rule. nil when the cameras cover no finite extent.
    public init?(cameras: [Int: Camera], outputWidth: Int,
                 projection: PanoProjection = .spherical) {
        var tMin = Double.infinity, tMax = -Double.infinity
        var pMin = Double.infinity, pMax = -Double.infinity
        var uLo = Double.infinity, uHi = -Double.infinity
        var vLo = Double.infinity, vHi = -Double.infinity
        for (_, cam) in cameras {
            let w = Double(cam.width), h = Double(cam.height)
            let steps = 16
            for k in 0...steps {
                let t = Double(k) / Double(steps)
                for p in [SIMD2(t * w, 0), SIMD2(t * w, h), SIMD2(0, t * h), SIMD2(w, t * h)] {
                    let d = cam.ray(cam.centered(p))
                    // Yaw about the world y axis; elevation is asin(−y)
                    // because camera y points down.
                    let theta = atan2(d.x, d.z)
                    let phi = asin(max(-1, min(1, -d.y)))
                    tMin = min(tMin, theta)
                    tMax = max(tMax, theta)
                    pMin = min(pMin, phi)
                    pMax = max(pMax, phi)
                    let uv = projection.forward(theta: theta, phi: phi)
                    uLo = min(uLo, uv.x)
                    uHi = max(uHi, uv.x)
                    vLo = min(vLo, uv.y)
                    vHi = max(vHi, uv.y)
                }
            }
        }
        guard uHi > uLo, vHi > vLo else { return nil }
        self.projection = projection
        thetaMin = tMin
        thetaMax = tMax
        phiMin = pMin
        phiMax = pMax
        uMin = uLo
        uMax = uHi
        vMin = vLo
        vMax = vHi
        scale = Double(outputWidth) / (uHi - uLo)
        width = outputWidth
        height = max(1, Int((vHi - vLo) * scale))
    }

    private init(copying g: PanoGeometry, outputWidth: Int) {
        projection = g.projection
        thetaMin = g.thetaMin
        thetaMax = g.thetaMax
        phiMin = g.phiMin
        phiMax = g.phiMax
        uMin = g.uMin
        uMax = g.uMax
        vMin = g.vMin
        vMax = g.vMax
        scale = Double(outputWidth) / (g.uMax - g.uMin)
        width = outputWidth
        height = max(1, Int((g.vMax - g.vMin) * scale))
    }

    /// Same extent and projection at a different output resolution.
    public func scaled(toWidth newWidth: Int) -> PanoGeometry {
        PanoGeometry(copying: self, outputWidth: newWidth)
    }

    /// World ray direction for a pano pixel (pixel centers at +0.5).
    public func direction(px: Double, py: Double) -> SIMD3<Double> {
        let u = uMin + (px + 0.5) / scale
        let v = vMax - (py + 0.5) / scale   // row 0 is the top of the frame
        let (theta, phi) = projection.inverse(u: u, v: v)
        let cosPhi = cos(phi)
        return SIMD3(sin(theta) * cosPhi, -sin(phi), cos(theta) * cosPhi)
    }

    /// Pano pixel for a world ray direction.
    public func panoPoint(direction d: SIMD3<Double>) -> SIMD2<Double> {
        let theta = atan2(d.x, d.z)
        let phi = asin(max(-1, min(1, -d.y)))
        let uv = projection.forward(theta: theta, phi: phi)
        return SIMD2((uv.x - uMin) * scale - 0.5, (vMax - uv.y) * scale - 0.5)
    }
}

/// One image projected into pano space: an RGB patch over its bounding box,
/// with validity (1 where the image covers the pixel) and a tent weight that
/// falls off linearly toward the source-image edges.
public struct ImageLayer {
    public var imageIndex: Int
    /// Top-left of the patch in output-space pixels.
    public var x0: Int
    public var y0: Int
    /// Resampled colour over the bounding box; undefined where validity is 0.
    public var rgb: RGBImage
    /// 1 where the source image covers the pixel, else 0.
    public var validity: ImageF
    /// Center-peaked weight (1 at the source center, ~0 at its edges), used by
    /// the seam finder's locality term.
    public var tent: ImageF

    public var width: Int { rgb.width }
    public var height: Int { rgb.height }
}

/// Inverse-mapping renderer for one camera: every pano pixel in the image's
/// bounding box is turned into a ray, projected into the camera, pulled back
/// through the parallax mesh if there is one, and bilinearly sampled.
public enum LayerProjector {

    /// Projects one source image into pano space over its bounding box.
    /// `image` may be at any resolution; `camera` is at registration scale.
    public static func project(imageIndex: Int,
                               camera: Camera,
                               image: RGBImage,
                               mesh: WarpMesh?,
                               geometry: PanoGeometry) -> ImageLayer? {
        // Bounding box from the border ring, padded for the mesh warp.
        // 24 samples per edge (more than PanoGeometry's 16) because a tight
        // box matters here: every pixel in it is rendered.
        let w = Double(camera.width), h = Double(camera.height)
        var xMin = Double.infinity, xMax = -Double.infinity
        var yMin = Double.infinity, yMax = -Double.infinity
        let steps = 24
        for k in 0...steps {
            let t = Double(k) / Double(steps)
            for p in [SIMD2(t * w, 0), SIMD2(t * w, h), SIMD2(0, t * h), SIMD2(w, t * h)] {
                let pt = geometry.panoPoint(direction: camera.ray(camera.centered(p)))
                xMin = min(xMin, pt.x)
                xMax = max(xMax, pt.x)
                yMin = min(yMin, pt.y)
                yMax = max(yMax, pt.y)
            }
        }
        // The mesh moves source pixels by up to maxOffset (source px); at
        // scale/focal pano px per source px that is how far the image can
        // reach past its rigid outline. Plus 2 for the bilinear footprint.
        let pad = ((mesh?.maxOffset ?? 0) * geometry.scale / camera.focal) + 2
        let x0 = max(0, Int(xMin - pad))
        let y0 = max(0, Int(yMin - pad))
        let x1 = min(geometry.width - 1, Int(xMax + pad))
        let y1 = min(geometry.height - 1, Int(yMax + pad))
        guard x1 > x0, y1 > y0 else { return nil }

        let lw = x1 - x0 + 1, lh = y1 - y0 + 1
        var rgb = RGBImage(width: lw, height: lh)
        var validity = ImageF(width: lw, height: lh)
        var tent = ImageF(width: lw, height: lh)

        // Camera is at registration scale; the image may be full resolution.
        let imgScaleX = Double(image.width) / Double(camera.width)
        let imgScaleY = Double(image.height) / Double(camera.height)

        // Row-parallel; each output row is written by exactly one thread.
        rgb.r.pixels.withUnsafeMutableBufferPointer { rp in
        rgb.g.pixels.withUnsafeMutableBufferPointer { gp in
        rgb.b.pixels.withUnsafeMutableBufferPointer { bp in
        validity.pixels.withUnsafeMutableBufferPointer { vp in
        tent.pixels.withUnsafeMutableBufferPointer { tp in
            DispatchQueue.concurrentPerform(iterations: lh) { row in
                let py = y0 + row
                for col in 0..<lw {
                    let d = geometry.direction(px: Double(x0 + col), py: Double(py))
                    guard let p = camera.project(d) else { continue }
                    // Centered pixel back to top-left origin.
                    var px = p.x + Double(camera.width) / 2
                    var pyi = p.y + Double(camera.height) / 2
                    // The bundle-adjusted camera describes the mesh-corrected
                    // image; the actual source pixel is u ≈ p − d(p).
                    if let mesh {
                        let off = mesh.offset(x: px, y: pyi)
                        px -= off.x
                        pyi -= off.y
                    }
                    guard px >= 0, px < Double(camera.width) - 1,
                          pyi >= 0, pyi < Double(camera.height) - 1 else { continue }
                    let c = image.sample(x: Float(px * imgScaleX), y: Float(pyi * imgScaleY))
                    let i = row * lw + col
                    rp[i] = c.x
                    gp[i] = c.y
                    bp[i] = c.z
                    vp[i] = 1
                    // Tent weight, floored so a pixel covered by only one
                    // image never carries zero weight.
                    let wx = 1 - abs(2 * px / Double(camera.width) - 1)
                    let wy = 1 - abs(2 * pyi / Double(camera.height) - 1)
                    tp[i] = Float(max(wx * wy, 1e-5))
                }
            }
        }}}}}

        return ImageLayer(imageIndex: imageIndex, x0: x0, y0: y0,
                          rgb: rgb, validity: validity, tent: tent)
    }
}
