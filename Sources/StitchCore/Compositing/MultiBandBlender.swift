import Accelerate
import Foundation

/// Burt-Adelson multi-band blending (Brown & Lowe §7): each image's Laplacian
/// pyramid is accumulated with weights from its seam mask's Gaussian pyramid,
/// so low frequencies blend over large ranges and high frequencies over short
/// ones. Validity-normalized pyramids keep invalid regions from bleeding dark.
///
/// Each image is processed on a pyramid over its own bounding box, aligned to
/// 2^(levels-1) so tile grids coincide exactly with the global accumulator
/// grids at every level; global buffers are padded to the same alignment.
public final class MultiBandBlender {
    private let levels: Int
    private let outWidth: Int
    private let outHeight: Int
    private let align: Int
    private let sizes: [(w: Int, h: Int)]   // padded global sizes per level
    private var numR: [ImageF] = []
    private var numG: [ImageF] = []
    private var numB: [ImageF] = []
    private var den: [ImageF] = []

    private static let eps: Float = 1e-5
    private static let pyramidSigma: Float = 2.0

    public init(width: Int, height: Int, levels: Int = 5) {
        self.levels = levels
        outWidth = width
        outHeight = height
        align = 1 << (levels - 1)
        let pw = (width + align - 1) / align * align
        let ph = (height + align - 1) / align * align
        var s: [(Int, Int)] = []
        for l in 0..<levels {
            s.append((pw >> l, ph >> l))
            numR.append(ImageF(width: pw >> l, height: ph >> l))
            numG.append(ImageF(width: pw >> l, height: ph >> l))
            numB.append(ImageF(width: pw >> l, height: ph >> l))
            den.append(ImageF(width: pw >> l, height: ph >> l))
        }
        sizes = s
    }

    // MARK: - Level helpers

    private static func shrink(_ img: ImageF) -> ImageF {
        let blurred = Convolution.gaussianBlur(img, sigma: pyramidSigma)
        return Convolution.resize(blurred, width: img.width / 2, height: img.height / 2)
    }

    /// a ./ max(b, eps), forced to zero where b <= eps. Fully vectorized.
    private static func normalized(_ a: ImageF, by b: ImageF) -> ImageF {
        var clipped = [Float](repeating: 0, count: b.pixels.count)
        vDSP.clip(b.pixels, to: eps...Float.greatestFiniteMagnitude, result: &clipped)
        var out = ImageF(width: a.width, height: a.height)
        vDSP.divide(a.pixels, clipped, result: &out.pixels)
        // Zero where b <= eps: mask = thresholded(b)/clipped ∈ {0, 1}.
        var mask = [Float](repeating: 0, count: b.pixels.count)
        vDSP.threshold(b.pixels, to: eps, with: .zeroFill, result: &mask)
        vDSP.divide(mask, clipped, result: &mask)
        vDSP.multiply(out.pixels, mask, result: &out.pixels)
        return out
    }

    // MARK: - Accumulation

    /// Adds one image: `rgb`/`validity` are the layer patch, `seamMask` the 0/1
    /// seam ownership over the same patch, placed at (x0, y0) in pano space.
    public func add(rgb: RGBImage, validity: ImageF, seamMask: ImageF, x0: Int, y0: Int) {
        // Tile bounding box aligned so tile grids match global grids per level.
        let gx0 = x0 / align * align
        let gy0 = y0 / align * align
        let dx = x0 - gx0, dy = y0 - gy0
        let lw = (dx + rgb.width + align - 1) / align * align
        let lh = (dy + rgb.height + align - 1) / align * align

        var cR = ImageF(width: lw, height: lh)
        var cG = ImageF(width: lw, height: lh)
        var cB = ImageF(width: lw, height: lh)
        var cV = ImageF(width: lw, height: lh)
        var cW = ImageF(width: lw, height: lh)
        for row in 0..<rgb.height {
            let ti = (dy + row) * lw + dx
            let li = row * rgb.width
            for col in 0..<rgb.width {
                let v = validity.pixels[li + col]
                guard v > 0 else { continue }
                cR.pixels[ti + col] = rgb.r.pixels[li + col] * v
                cG.pixels[ti + col] = rgb.g.pixels[li + col] * v
                cB.pixels[ti + col] = rgb.b.pixels[li + col] * v
                cV.pixels[ti + col] = v
                cW.pixels[ti + col] = seamMask.pixels[li + col]
            }
        }

        for l in 0..<levels {
            let curW = lw >> l, curH = lh >> l
            let isLast = l == levels - 1

            // Weight active only where this level still has validity support.
            var wEff = ImageF(width: curW, height: curH)
            var vMask = [Float](repeating: 0, count: curW * curH)
            vDSP.threshold(cV.pixels, to: Self.eps, with: .zeroFill, result: &vMask)
            var vClip = [Float](repeating: 0, count: curW * curH)
            vDSP.clip(cV.pixels, to: Self.eps...Float.greatestFiniteMagnitude, result: &vClip)
            vDSP.divide(vMask, vClip, result: &vMask)   // binary validity
            vDSP.multiply(cW.pixels, vMask, result: &wEff.pixels)

            let iR = Self.normalized(cR, by: cV)
            let iG = Self.normalized(cG, by: cV)
            let iB = Self.normalized(cB, by: cV)

            var bandR = iR, bandG = iG, bandB = iB
            var nextR = cR, nextG = cG, nextB = cB, nextV = cV, nextW = cW
            if !isLast {
                nextR = Self.shrink(cR)
                nextG = Self.shrink(cG)
                nextB = Self.shrink(cB)
                nextV = Self.shrink(cV)
                nextW = Self.shrink(cW)
                let upR = Convolution.resize(Self.normalized(nextR, by: nextV), width: curW, height: curH)
                let upG = Convolution.resize(Self.normalized(nextG, by: nextV), width: curW, height: curH)
                let upB = Convolution.resize(Self.normalized(nextB, by: nextV), width: curW, height: curH)
                vDSP.subtract(iR.pixels, upR.pixels, result: &bandR.pixels)
                vDSP.subtract(iG.pixels, upG.pixels, result: &bandG.pixels)
                vDSP.subtract(iB.pixels, upB.pixels, result: &bandB.pixels)
            }

            accumulate(band: bandR, weight: wEff, into: &numR[l], level: l,
                       ox: gx0 >> l, oy: gy0 >> l, tileW: curW, tileH: curH)
            accumulate(band: bandG, weight: wEff, into: &numG[l], level: l,
                       ox: gx0 >> l, oy: gy0 >> l, tileW: curW, tileH: curH)
            accumulate(band: bandB, weight: wEff, into: &numB[l], level: l,
                       ox: gx0 >> l, oy: gy0 >> l, tileW: curW, tileH: curH)
            accumulate(band: nil, weight: wEff, into: &den[l], level: l,
                       ox: gx0 >> l, oy: gy0 >> l, tileW: curW, tileH: curH)

            cR = nextR; cG = nextG; cB = nextB; cV = nextV; cW = nextW
        }
    }

    /// Row-wise vDSP accumulation of band·weight (or weight alone) into a
    /// global level buffer at tile offset (ox, oy), clipped to global bounds.
    private func accumulate(band: ImageF?, weight: ImageF, into target: inout ImageF,
                            level: Int, ox: Int, oy: Int, tileW: Int, tileH: Int) {
        let gw = sizes[level].w, gh = sizes[level].h
        let cols = min(tileW, gw - ox)
        guard cols > 0 else { return }
        var product = [Float](repeating: 0, count: tileW * tileH)
        if let band {
            vDSP.multiply(band.pixels, weight.pixels, result: &product)
        } else {
            product = weight.pixels
        }
        product.withUnsafeBufferPointer { pp in
            target.pixels.withUnsafeMutableBufferPointer { tp in
                for row in 0..<tileH {
                    let gy = oy + row
                    guard gy >= 0, gy < gh else { continue }
                    let src = pp.baseAddress! + row * tileW
                    let dst = tp.baseAddress! + gy * gw + ox
                    vDSP_vadd(src, 1, dst, 1, dst, 1, vDSP_Length(cols))
                }
            }
        }
    }

    // MARK: - Output

    /// Collapses the accumulated pyramid into the final panorama.
    public func finalize() -> RGBImage {
        var outR = Self.normalized(numR[levels - 1], by: den[levels - 1])
        var outG = Self.normalized(numG[levels - 1], by: den[levels - 1])
        var outB = Self.normalized(numB[levels - 1], by: den[levels - 1])
        for l in stride(from: levels - 2, through: 0, by: -1) {
            outR = Convolution.resize(outR, width: sizes[l].w, height: sizes[l].h)
            outG = Convolution.resize(outG, width: sizes[l].w, height: sizes[l].h)
            outB = Convolution.resize(outB, width: sizes[l].w, height: sizes[l].h)
            let bandR = Self.normalized(numR[l], by: den[l])
            let bandG = Self.normalized(numG[l], by: den[l])
            let bandB = Self.normalized(numB[l], by: den[l])
            vDSP.add(outR.pixels, bandR.pixels, result: &outR.pixels)
            vDSP.add(outG.pixels, bandG.pixels, result: &outG.pixels)
            vDSP.add(outB.pixels, bandB.pixels, result: &outB.pixels)
        }
        vDSP.clip(outR.pixels, to: 0...1, result: &outR.pixels)
        vDSP.clip(outG.pixels, to: 0...1, result: &outG.pixels)
        vDSP.clip(outB.pixels, to: 0...1, result: &outB.pixels)
        // Black out uncovered pixels, then trim alignment padding.
        for i in 0..<(sizes[0].w * sizes[0].h) where den[0].pixels[i] <= Self.eps {
            outR.pixels[i] = 0
            outG.pixels[i] = 0
            outB.pixels[i] = 0
        }
        return RGBImage(r: outR, g: outG, b: outB)
            .cropped(x0: 0, y0: 0, width: outWidth, height: outHeight)
    }

    /// Coverage mask at output resolution (1 where any image contributed).
    public var coverage: ImageF {
        var m = ImageF(width: outWidth, height: outHeight)
        let gw = sizes[0].w
        for y in 0..<outHeight {
            for x in 0..<outWidth where den[0].pixels[y * gw + x] > Self.eps {
                m.pixels[y * outWidth + x] = 1
            }
        }
        return m
    }
}
