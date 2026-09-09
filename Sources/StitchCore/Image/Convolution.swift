import Accelerate
// Low-level image filtering shared by the SIFT pyramid, the multi-band
// blender's Laplacian pyramids, and registration-time resizing. All of the
// per-pixel heavy lifting goes through Accelerate (vImage, vDSP).

import Foundation

/// Separable Gaussian filtering and pyramid resampling on planar float images,
/// built on vImage.
public enum Convolution {

    /// Normalized 1-D Gaussian taps, odd length, truncated at ±4σ (about
    /// 0.006% of the mass lost per tail, invisible next to float error).
    public static func gaussianKernel(sigma: Float) -> [Float] {
        precondition(sigma > 0)
        let radius = max(1, Int((sigma * 4).rounded(.up)))
        var kernel = (0...(2 * radius)).map { i -> Float in
            let d = Float(i - radius)
            return expf(-d * d / (2 * sigma * sigma))
        }
        let sum = kernel.reduce(0, +)
        for i in kernel.indices { kernel[i] /= sum }
        return kernel
    }

    /// Gaussian blur via two separable vImage convolution passes (edge-extend).
    /// Edge-extend replicates the border pixel outward, which keeps the
    /// pyramid's outer rows from darkening the way zero-padding would.
    public static func gaussianBlur(_ image: ImageF, sigma: Float) -> ImageF {
        // Sub-0.01 σ is a one-tap kernel; skip the two passes entirely.
        guard sigma > 0.01 else { return image }
        let kernel = gaussianKernel(sigma: sigma)
        let w = image.width, h = image.height
        var src = image
        var tmp = ImageF(width: w, height: h)
        var dst = ImageF(width: w, height: h)
        let rowBytes = w * MemoryLayout<Float>.size

        kernel.withUnsafeBufferPointer { k in
            src.pixels.withUnsafeMutableBufferPointer { sp in
                tmp.pixels.withUnsafeMutableBufferPointer { tp in
                    var sbuf = vImage_Buffer(data: sp.baseAddress!, height: vImagePixelCount(h),
                                             width: vImagePixelCount(w), rowBytes: rowBytes)
                    var tbuf = vImage_Buffer(data: tp.baseAddress!, height: vImagePixelCount(h),
                                             width: vImagePixelCount(w), rowBytes: rowBytes)
                    // Horizontal pass: 1 x N kernel.
                    vImageConvolve_PlanarF(&sbuf, &tbuf, nil, 0, 0,
                                           k.baseAddress!, 1, UInt32(kernel.count),
                                           0, vImage_Flags(kvImageEdgeExtend))
                }
            }
            tmp.pixels.withUnsafeMutableBufferPointer { tp in
                dst.pixels.withUnsafeMutableBufferPointer { dp in
                    var tbuf = vImage_Buffer(data: tp.baseAddress!, height: vImagePixelCount(h),
                                             width: vImagePixelCount(w), rowBytes: rowBytes)
                    var dbuf = vImage_Buffer(data: dp.baseAddress!, height: vImagePixelCount(h),
                                             width: vImagePixelCount(w), rowBytes: rowBytes)
                    // Vertical pass: N x 1 kernel.
                    vImageConvolve_PlanarF(&tbuf, &dbuf, nil, 0, 0,
                                           k.baseAddress!, UInt32(kernel.count), 1,
                                           0, vImage_Flags(kvImageEdgeExtend))
                }
            }
        }
        return dst
    }

    /// Decimate by 2 (every other pixel). Caller is responsible for pre-blurring.
    /// The SIFT pyramid decimates its level-S image, already at 2σ₀, which is
    /// exactly the anti-aliasing the next octave needs (Lowe §3).
    public static func downsample2(_ image: ImageF) -> ImageF {
        let w = max(1, image.width / 2), h = max(1, image.height / 2)
        var out = ImageF(width: w, height: h)
        for y in 0..<h {
            let srcRow = (2 * y) * image.width
            let dstRow = y * w
            for x in 0..<w {
                out.pixels[dstRow + x] = image.pixels[srcRow + 2 * x]
            }
        }
        return out
    }

    /// Bilinear 2x upsample (used when SIFT doubles the input image).
    public static func upsample2(_ image: ImageF) -> ImageF {
        let w = image.width * 2, h = image.height * 2
        var out = ImageF(width: w, height: h)
        for y in 0..<h {
            for x in 0..<w {
                out[x, y] = image.sample(x: Float(x) * 0.5, y: Float(y) * 0.5)
            }
        }
        return out
    }

    /// Bilinear-quality resampling via vImage. Used for arbitrary ratios
    /// (registration caps, debug thumbnails); the pyramids use the exact
    /// 2× routines above.
    public static func resize(_ image: ImageF, width: Int, height: Int) -> ImageF {
        if image.width == width && image.height == height { return image }
        var src = image
        var dst = ImageF(width: width, height: height)
        src.pixels.withUnsafeMutableBufferPointer { sp in
            dst.pixels.withUnsafeMutableBufferPointer { dp in
                var sbuf = vImage_Buffer(data: sp.baseAddress!, height: vImagePixelCount(image.height),
                                         width: vImagePixelCount(image.width),
                                         rowBytes: image.width * MemoryLayout<Float>.size)
                var dbuf = vImage_Buffer(data: dp.baseAddress!, height: vImagePixelCount(height),
                                         width: vImagePixelCount(width),
                                         rowBytes: width * MemoryLayout<Float>.size)
                vImageScale_PlanarF(&sbuf, &dbuf, nil, vImage_Flags(kvImageEdgeExtend))
            }
        }
        return dst
    }

    /// a − b, elementwise. Builds the DoG layers and the Laplacian pyramid.
    public static func subtract(_ a: ImageF, _ b: ImageF) -> ImageF {
        precondition(a.width == b.width && a.height == b.height)
        var out = ImageF(width: a.width, height: a.height)
        vDSP.subtract(a.pixels, b.pixels, result: &out.pixels)
        return out
    }
}
