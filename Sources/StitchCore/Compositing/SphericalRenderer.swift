// Milestone-4 preview renderer, kept for `stitch align` style inspection of
// registration alone. The production path is Compositor over PanoGeometry;
// this one has no gain, seams, or pyramids, so alignment errors show up as
// plain ghosting.

import Foundation
import simd

/// Renders aligned cameras into spherical (θ, φ) coordinates with linear
/// (tent-weighted) blending, the paper's eq. 30. This is the milestone-4
/// preview renderer; gain compensation, graph-cut seams, and multi-band
/// blending replace the blend stage later.
public enum SphericalRenderer {

    /// The rendered equirectangular image and the angular window it covers.
    public struct Output {
        public var image: RGBImage
        /// Yaw extent, radians; θ = atan2(x, z) of the world ray.
        public var thetaRange: ClosedRange<Double>
        /// Elevation extent, radians; φ = asin(−y), positive upward (camera y is down).
        public var phiRange: ClosedRange<Double>
    }

    /// Blends every camera that has an image into one equirectangular frame
    /// `outputWidth` pixels wide; height follows from the angular aspect.
    /// `images` may be at a different resolution than the cameras (which are
    /// at registration scale). nil if there is nothing to render.
    public static func render(cameras: [Int: Camera],
                              images: [Int: RGBImage],
                              meshes: [Int: WarpMesh] = [:],
                              outputWidth: Int = 4000) -> Output? {
        guard !cameras.isEmpty else { return nil }

        // Angular extent: project a ring of border pixels from every image.
        // Straight image edges curve on the sphere, so the corners alone
        // would undershoot; 16 samples per edge is plenty for a bounding box.
        var thetaMin = Double.infinity, thetaMax = -Double.infinity
        var phiMin = Double.infinity, phiMax = -Double.infinity
        for (_, cam) in cameras {
            let w = Double(cam.width), h = Double(cam.height)
            let steps = 16
            var border: [SIMD2<Double>] = []
            for k in 0...steps {
                let t = Double(k) / Double(steps)
                border.append(SIMD2(t * w, 0))
                border.append(SIMD2(t * w, h))
                border.append(SIMD2(0, t * h))
                border.append(SIMD2(w, t * h))
            }
            for p in border {
                let d = cam.ray(cam.centered(p))
                let theta = atan2(d.x, d.z)
                let phi = asin(max(-1, min(1, -d.y)))
                thetaMin = min(thetaMin, theta)
                thetaMax = max(thetaMax, theta)
                phiMin = min(phiMin, phi)
                phiMax = max(phiMax, phi)
            }
        }
        guard thetaMax > thetaMin, phiMax > phiMin else { return nil }

        // Equirectangular: uniform pixels per radian in both axes.
        let scale = Double(outputWidth) / (thetaMax - thetaMin)
        let outputHeight = max(1, Int((phiMax - phiMin) * scale))
        var out = RGBImage(width: outputWidth, height: outputHeight)

        let camList = cameras.keys.sorted().compactMap { key -> (Camera, RGBImage, WarpMesh?)? in
            guard let img = images[key] else { return nil }
            return (cameras[key]!, img, meshes[key])
        }

        // Row-parallel accumulation; each row is written by exactly one thread.
        out.r.pixels.withUnsafeMutableBufferPointer { rp in
            out.g.pixels.withUnsafeMutableBufferPointer { gp in
                out.b.pixels.withUnsafeMutableBufferPointer { bp in
                    DispatchQueue.concurrentPerform(iterations: outputHeight) { y in
                        // Row 0 is the top of the frame, so φ decreases with y.
                        let phi = phiMax - (Double(y) + 0.5) / scale
                        let cosPhi = cos(phi), sinPhi = sin(phi)
                        for x in 0..<outputWidth {
                            let theta = thetaMin + (Double(x) + 0.5) / scale
                            // Inverse of (θ, φ) above: unit ray in world space.
                            let d = SIMD3(sin(theta) * cosPhi, -sinPhi, cos(theta) * cosPhi)
                            var acc = SIMD3<Float>.zero
                            var wSum: Float = 0
                            for (cam, img, mesh) in camList {
                                guard let p = cam.project(d) else { continue }
                                var px = p.x + Double(cam.width) / 2
                                var py = p.y + Double(cam.height) / 2
                                // The global model addresses corrected space;
                                // pull back through the parallax mesh to find
                                // the actual source pixel: u ≈ p − d(p).
                                if let mesh {
                                    let off = mesh.offset(x: px, y: py)
                                    px -= off.x
                                    py -= off.y
                                }
                                guard px >= 0, px < Double(cam.width) - 1,
                                      py >= 0, py < Double(cam.height) - 1 else { continue }
                                // Tent weight: 1 at center, 0 at the edges.
                                let wx = 1 - abs(2 * px / Double(cam.width) - 1)
                                let wy = 1 - abs(2 * py / Double(cam.height) - 1)
                                let weight = Float(wx * wy)
                                // Registration ran on grayscale of possibly different
                                // scale; map through relative size.
                                let sx = Float(px) * Float(img.width) / Float(cam.width)
                                let sy = Float(py) * Float(img.height) / Float(cam.height)
                                acc += img.sample(x: sx, y: sy) * weight
                                wSum += weight
                            }
                            if wSum > 0 {
                                let i = y * outputWidth + x
                                let c = acc / wSum
                                rp[i] = c.x
                                gp[i] = c.y
                                bp[i] = c.z
                            }
                        }
                    }
                }
            }
        }
        return Output(image: out, thetaRange: thetaMin...thetaMax, phiRange: phiMin...phiMax)
    }
}
