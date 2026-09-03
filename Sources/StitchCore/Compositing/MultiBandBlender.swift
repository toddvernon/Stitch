import Accelerate
import Foundation

/// Burt-Adelson multi-band blending (Brown & Lowe §7): each image's Laplacian
/// pyramid is accumulated with weights from its seam mask's Gaussian pyramid,
/// so low frequencies blend over large ranges and high frequencies over short
/// ones. Validity-normalized pyramids keep invalid regions from bleeding dark.
public final class MultiBandBlender {
    private let levels: Int
    private var sizes: [(w: Int, h: Int)] = []
    private var numR: [ImageF] = []
    private var numG: [ImageF] = []
    private var numB: [ImageF] = []
    private var den: [ImageF] = []

    public init(width: Int, height: Int, levels: Int = 5) {
        self.levels = levels
        var w = width, h = height
        for _ in 0..<levels {
            sizes.append((w, h))
            numR.append(ImageF(width: w, height: h))
            numG.append(ImageF(width: w, height: h))
            numB.append(ImageF(width: w, height: h))
            den.append(ImageF(width: w, height: h))
            w = max(1, (w + 1) / 2)
            h = max(1, (h + 1) / 2)
        }
    }

    private static let pyramidSigma: Float = 2.0

    private static func shrink(_ img: ImageF, to size: (w: Int, h: Int)) -> ImageF {
        let blurred = Convolution.gaussianBlur(img, sigma: pyramidSigma)
        return resize(blurred, width: size.w, height: size.h)
    }

    static func resize(_ img: ImageF, width: Int, height: Int) -> ImageF {
        if img.width == width && img.height == height { return img }
        var out = ImageF(width: width, height: height)
        let sx = Float(img.width) / Float(width)
        let sy = Float(img.height) / Float(height)
        for y in 0..<height {
            for x in 0..<width {
                out[x, y] = img.sample(x: (Float(x) + 0.5) * sx - 0.5,
                                       y: (Float(y) + 0.5) * sy - 0.5)
            }
        }
        return out
    }

    private static func multiply(_ a: ImageF, _ b: ImageF) -> ImageF {
        var out = ImageF(width: a.width, height: a.height)
        vDSP.multiply(a.pixels, b.pixels, result: &out.pixels)
        return out
    }

    /// Element-wise a / max(b, eps), zero where b ≈ 0.
    private static func normalized(_ a: ImageF, by b: ImageF) -> ImageF {
        var out = ImageF(width: a.width, height: a.height)
        for i in a.pixels.indices {
            let d = b.pixels[i]
            out.pixels[i] = d > 1e-5 ? a.pixels[i] / d : 0
        }
        return out
    }

    /// Adds one image: `rgb`/`validity` are the layer patch, `seamMask` is the
    /// 0/1 seam ownership over the same patch. `x0`,`y0` place the patch.
    public func add(rgb: RGBImage, validity: ImageF, seamMask: ImageF, x0: Int, y0: Int) {
        // Place the patch into full-pano canvases (premultiplied by validity).
        var canvasR = ImageF(width: sizes[0].w, height: sizes[0].h)
        var canvasG = ImageF(width: sizes[0].w, height: sizes[0].h)
        var canvasB = ImageF(width: sizes[0].w, height: sizes[0].h)
        var canvasV = ImageF(width: sizes[0].w, height: sizes[0].h)
        var canvasW = ImageF(width: sizes[0].w, height: sizes[0].h)
        for row in 0..<rgb.height {
            let py = y0 + row
            guard py >= 0, py < sizes[0].h else { continue }
            for col in 0..<rgb.width {
                let px = x0 + col
                guard px >= 0, px < sizes[0].w else { continue }
                let li = row * rgb.width + col
                let v = validity.pixels[li]
                guard v > 0 else { continue }
                let pi = py * sizes[0].w + px
                canvasR.pixels[pi] = rgb.r.pixels[li] * v
                canvasG.pixels[pi] = rgb.g.pixels[li] * v
                canvasB.pixels[pi] = rgb.b.pixels[li] * v
                canvasV.pixels[pi] = v
                canvasW.pixels[pi] = seamMask.pixels[li]
            }
        }

        // Gaussian pyramids of premultiplied color, validity, and seam weight.
        var pR = [canvasR], pG = [canvasG], pB = [canvasB], pV = [canvasV], pW = [canvasW]
        for l in 1..<levels {
            pR.append(Self.shrink(pR[l - 1], to: sizes[l]))
            pG.append(Self.shrink(pG[l - 1], to: sizes[l]))
            pB.append(Self.shrink(pB[l - 1], to: sizes[l]))
            pV.append(Self.shrink(pV[l - 1], to: sizes[l]))
            pW.append(Self.shrink(pW[l - 1], to: sizes[l]))
        }

        // Validity-normalized image estimates per level.
        let iR = (0..<levels).map { Self.normalized(pR[$0], by: pV[$0]) }
        let iG = (0..<levels).map { Self.normalized(pG[$0], by: pV[$0]) }
        let iB = (0..<levels).map { Self.normalized(pB[$0], by: pV[$0]) }

        // Accumulate: band-pass (Laplacian) levels weighted by the blurred seam
        // mask; the coarsest level carries the low-pass remainder.
        for l in 0..<levels {
            let isLast = l == levels - 1
            let upR = isLast ? nil : Self.resize(iR[l + 1], width: sizes[l].w, height: sizes[l].h)
            let upG = isLast ? nil : Self.resize(iG[l + 1], width: sizes[l].w, height: sizes[l].h)
            let upB = isLast ? nil : Self.resize(iB[l + 1], width: sizes[l].w, height: sizes[l].h)
            let count = sizes[l].w * sizes[l].h
            for i in 0..<count {
                let w = pW[l].pixels[i]
                guard w > 1e-5, pV[l].pixels[i] > 1e-5 else { continue }
                let bandR = isLast ? iR[l].pixels[i] : iR[l].pixels[i] - upR!.pixels[i]
                let bandG = isLast ? iG[l].pixels[i] : iG[l].pixels[i] - upG!.pixels[i]
                let bandB = isLast ? iB[l].pixels[i] : iB[l].pixels[i] - upB!.pixels[i]
                numR[l].pixels[i] += bandR * w
                numG[l].pixels[i] += bandG * w
                numB[l].pixels[i] += bandB * w
                den[l].pixels[i] += w
            }
        }
    }

    /// Collapses the accumulated pyramid into the final panorama.
    public func finalize() -> RGBImage {
        var outR = Self.normalized(numR[levels - 1], by: den[levels - 1])
        var outG = Self.normalized(numG[levels - 1], by: den[levels - 1])
        var outB = Self.normalized(numB[levels - 1], by: den[levels - 1])
        for l in stride(from: levels - 2, through: 0, by: -1) {
            outR = Self.resize(outR, width: sizes[l].w, height: sizes[l].h)
            outG = Self.resize(outG, width: sizes[l].w, height: sizes[l].h)
            outB = Self.resize(outB, width: sizes[l].w, height: sizes[l].h)
            let bandR = Self.normalized(numR[l], by: den[l])
            let bandG = Self.normalized(numG[l], by: den[l])
            let bandB = Self.normalized(numB[l], by: den[l])
            for i in 0..<(sizes[l].w * sizes[l].h) {
                // Zero out pixels no image covers at the finest level.
                if l == 0 && den[0].pixels[i] <= 1e-5 {
                    outR.pixels[i] = 0
                    outG.pixels[i] = 0
                    outB.pixels[i] = 0
                } else {
                    outR.pixels[i] = min(max(outR.pixels[i] + bandR.pixels[i], 0), 1)
                    outG.pixels[i] = min(max(outG.pixels[i] + bandG.pixels[i], 0), 1)
                    outB.pixels[i] = min(max(outB.pixels[i] + bandB.pixels[i], 0), 1)
                }
            }
        }
        return RGBImage(r: outR, g: outG, b: outB)
    }

    /// Coverage mask at full resolution (1 where any image contributed).
    public var coverage: ImageF {
        var m = ImageF(width: sizes[0].w, height: sizes[0].h)
        for i in m.pixels.indices where den[0].pixels[i] > 1e-5 {
            m.pixels[i] = 1
        }
        return m
    }
}
