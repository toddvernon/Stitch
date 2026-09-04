import Foundation
import simd

/// Output space for a multi-viewpoint strip: the dominant (facade) plane
/// itself. Each image reaches it through one similarity, so the whole strip
/// is a flat mosaic — no sphere, no projection choice — and the seam finder
/// does the multi-viewpoint selection (Agarwala et al. 2006), preferring the
/// photo taken most directly in front of each region.
public struct StripGeometry: LayerSource {
    /// Registration-scale pixels of each image → strip frame.
    public let transforms: [Int: Similarity]
    /// Registration-scale image sizes.
    public let sizes: [Int: (width: Int, height: Int)]
    /// Strip-frame coordinate of the output's top-left corner.
    public let origin: SIMD2<Double>
    /// Strip-frame extent covered by the output.
    public let extent: SIMD2<Double>
    /// Output pixels per strip-frame unit.
    public let scale: Double
    public let width: Int
    public let height: Int

    public init?(transforms: [Int: Similarity],
                 sizes: [Int: (width: Int, height: Int)],
                 outputWidth: Int) {
        var lo = SIMD2<Double>(.infinity, .infinity)
        var hi = SIMD2<Double>(-.infinity, -.infinity)
        for (idx, t) in transforms {
            guard let size = sizes[idx] else { continue }
            let w = Double(size.width), h = Double(size.height)
            for corner in [SIMD2(0, 0), SIMD2(w, 0), SIMD2(0, h), SIMD2(w, h)] {
                let p = t.apply(corner)
                lo = pointwiseMin(lo, p)
                hi = pointwiseMax(hi, p)
            }
        }
        guard hi.x > lo.x, hi.y > lo.y else { return nil }
        self.init(transforms: transforms, sizes: sizes, origin: lo, extent: hi - lo,
                  outputWidth: outputWidth)
    }

    private init(transforms: [Int: Similarity], sizes: [Int: (width: Int, height: Int)],
                 origin: SIMD2<Double>, extent: SIMD2<Double>, outputWidth: Int) {
        self.transforms = transforms
        self.sizes = sizes
        self.origin = origin
        self.extent = extent
        scale = Double(outputWidth) / extent.x
        width = outputWidth
        height = max(1, Int(extent.y * scale))
    }

    public var imageIndices: [Int] { transforms.keys.sorted() }

    public func scaled(toWidth newWidth: Int) -> StripGeometry {
        StripGeometry(transforms: transforms, sizes: sizes, origin: origin, extent: extent,
                      outputWidth: newWidth)
    }

    /// Output px per registration px of the image, with sampling margin.
    public func sourceDimension(for imageIndex: Int) -> Int {
        let size = sizes[imageIndex]!
        let needed = Double(max(size.width, size.height)) * scale * transforms[imageIndex]!.scale * 1.2
        return Int(needed.rounded(.up))
    }

    /// Strip-frame point for an output pixel (pixel centers at +0.5).
    public func stripPoint(px: Double, py: Double) -> SIMD2<Double> {
        origin + SIMD2(px + 0.5, py + 0.5) / scale
    }

    /// Output pixel for a strip-frame point.
    public func outputPoint(_ p: SIMD2<Double>) -> SIMD2<Double> {
        (p - origin) * scale - SIMD2(0.5, 0.5)
    }

    public func project(imageIndex: Int, image: RGBImage) -> ImageLayer? {
        guard let t = transforms[imageIndex], let size = sizes[imageIndex] else { return nil }
        let w = Double(size.width), h = Double(size.height)

        var xMin = Double.infinity, xMax = -Double.infinity
        var yMin = Double.infinity, yMax = -Double.infinity
        for corner in [SIMD2(0, 0), SIMD2(w, 0), SIMD2(0, h), SIMD2(w, h)] {
            let p = outputPoint(t.apply(corner))
            xMin = min(xMin, p.x)
            xMax = max(xMax, p.x)
            yMin = min(yMin, p.y)
            yMax = max(yMax, p.y)
        }
        let x0 = max(0, Int(xMin - 2))
        let y0 = max(0, Int(yMin - 2))
        let x1 = min(width - 1, Int(xMax + 2))
        let y1 = min(height - 1, Int(yMax + 2))
        guard x1 > x0, y1 > y0 else { return nil }

        let lw = x1 - x0 + 1, lh = y1 - y0 + 1
        var rgb = RGBImage(width: lw, height: lh)
        var validity = ImageF(width: lw, height: lh)
        var tent = ImageF(width: lw, height: lh)

        let inv = t.inverse
        let imgScaleX = Double(image.width) / w
        let imgScaleY = Double(image.height) / h

        rgb.r.pixels.withUnsafeMutableBufferPointer { rp in
        rgb.g.pixels.withUnsafeMutableBufferPointer { gp in
        rgb.b.pixels.withUnsafeMutableBufferPointer { bp in
        validity.pixels.withUnsafeMutableBufferPointer { vp in
        tent.pixels.withUnsafeMutableBufferPointer { tp in
            DispatchQueue.concurrentPerform(iterations: lh) { row in
                let py = y0 + row
                for col in 0..<lw {
                    let s = inv.apply(stripPoint(px: Double(x0 + col), py: Double(py)))
                    guard s.x >= 0, s.x < w - 1, s.y >= 0, s.y < h - 1 else { continue }
                    let c = image.sample(x: Float(s.x * imgScaleX), y: Float(s.y * imgScaleY))
                    let i = row * lw + col
                    rp[i] = c.x
                    gp[i] = c.y
                    bp[i] = c.z
                    vp[i] = 1
                    let wx = 1 - abs(2 * s.x / w - 1)
                    let wy = 1 - abs(2 * s.y / h - 1)
                    tp[i] = Float(max(wx * wy, 1e-5))
                }
            }
        }}}}}

        return ImageLayer(imageIndex: imageIndex, x0: x0, y0: y0,
                          rgb: rgb, validity: validity, tent: tent)
    }
}
