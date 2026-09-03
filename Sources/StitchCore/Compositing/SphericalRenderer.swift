import Foundation
import simd

/// Renders aligned cameras into spherical (θ, φ) coordinates with linear
/// (tent-weighted) blending — the paper's eq. 30. This is the milestone-4
/// preview renderer; gain compensation, graph-cut seams, and multi-band
/// blending replace the blend stage later.
public enum SphericalRenderer {

    public struct Output {
        public var image: RGBImage
        public var thetaRange: ClosedRange<Double>
        public var phiRange: ClosedRange<Double>
    }

    public static func render(cameras: [Int: Camera],
                              images: [Int: RGBImage],
                              meshes: [Int: WarpMesh] = [:],
                              outputWidth: Int = 4000) -> Output? {
        guard !cameras.isEmpty else { return nil }

        // Angular extent: project a ring of border pixels from every image.
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
                        let phi = phiMax - (Double(y) + 0.5) / scale
                        let cosPhi = cos(phi), sinPhi = sin(phi)
                        for x in 0..<outputWidth {
                            let theta = thetaMin + (Double(x) + 0.5) / scale
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
