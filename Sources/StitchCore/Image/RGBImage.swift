import Accelerate
import CoreGraphics
import Foundation
import ImageIO

// Three-plane color image for the compositing stages (gain, seams, blend)
// and for output, plus the full-resolution RGB loader those stages use.
// Registration never touches color; it works on `ImageF` grayscale.

/// Planar float RGB image, values nominally [0, 1]. Three separate `ImageF`
/// planes rather than interleaved pixels, so every per-channel operation
/// (blur, pyramid, gain) reuses the single-channel code unchanged.
public struct RGBImage {
    public var r: ImageF
    public var g: ImageF
    public var b: ImageF

    public var width: Int { r.width }
    public var height: Int { r.height }

    public init(width: Int, height: Int) {
        r = ImageF(width: width, height: height)
        g = ImageF(width: width, height: height)
        b = ImageF(width: width, height: height)
    }

    public init(r: ImageF, g: ImageF, b: ImageF) {
        self.r = r
        self.g = g
        self.b = b
    }

    /// Bilinear sample of all three planes; see `ImageF.sample` for the
    /// coordinate convention and border clamping.
    public func sample(x: Float, y: Float) -> SIMD3<Float> {
        SIMD3(r.sample(x: x, y: y), g.sample(x: x, y: y), b.sample(x: x, y: y))
    }

    /// Copies out a sub-rectangle, which must lie inside the image. Callers
    /// are the compositor's auto-crop and the blender's final trim.
    public func cropped(x0: Int, y0: Int, width: Int, height: Int) -> RGBImage {
        var out = RGBImage(width: width, height: height)
        for y in 0..<height {
            for x in 0..<width {
                let si = (y0 + y) * self.width + (x0 + x)
                let di = y * width + x
                out.r.pixels[di] = r.pixels[si]
                out.g.pixels[di] = g.pixels[si]
                out.b.pixels[di] = b.pixels[si]
            }
        }
        return out
    }

    /// 8-bit RGBA CGImage for display or `ImageLoader.writeImage`. Values
    /// are clamped to [0, 1]; alpha is fully opaque.
    public func makeCGImage() -> CGImage {
        let w = width, h = height
        var rgba = [UInt8](repeating: 255, count: w * h * 4)
        for i in 0..<(w * h) {
            rgba[i * 4] = UInt8(min(max(r.pixels[i] * 255, 0), 255))
            rgba[i * 4 + 1] = UInt8(min(max(g.pixels[i] * 255, 0), 255))
            rgba[i * 4 + 2] = UInt8(min(max(b.pixels[i] * 255, 0), 255))
        }
        // makeImage copies the bitmap, so the CGImage outlives `rgba`.
        return rgba.withUnsafeMutableBytes { buf in
            CGContext(data: buf.baseAddress, width: w, height: h,
                      bitsPerComponent: 8, bytesPerRow: w * 4,
                      space: CGColorSpaceCreateDeviceRGB(),
                      bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue)!
                .makeImage()!
        }
    }
}

extension ImageLoader {

    /// Loads an image as planar float RGB, honoring EXIF orientation, with the
    /// same downsampling contract as `loadGrayscale`.
    public static func loadRGB(url: URL, maxDimension: Int? = nil) throws -> RGBImage {
        guard let source = CGImageSourceCreateWithURL(url as CFURL, nil) else {
            throw ImageLoaderError.cannotOpen(url)
        }
        var options: [CFString: Any] = [
            kCGImageSourceCreateThumbnailFromImageAlways: true,
            kCGImageSourceCreateThumbnailWithTransform: true,
            kCGImageSourceShouldCacheImmediately: true,
        ]
        if let maxDimension {
            options[kCGImageSourceThumbnailMaxPixelSize] = maxDimension
        } else if let props = CGImageSourceCopyPropertiesAtIndex(source, 0, nil) as? [CFString: Any],
                  let w = props[kCGImagePropertyPixelWidth] as? Int,
                  let h = props[kCGImagePropertyPixelHeight] as? Int {
            // The thumbnail path is the only one that applies the EXIF
            // transform, so request it at full size rather than falling
            // back to CreateImageAtIndex, which would come out unrotated.
            options[kCGImageSourceThumbnailMaxPixelSize] = max(w, h)
        }
        guard let cgImage = CGImageSourceCreateThumbnailAtIndex(source, 0, options as CFDictionary)
            ?? CGImageSourceCreateImageAtIndex(source, 0, nil) else {
            throw ImageLoaderError.cannotDecode(url)
        }

        // Draw into a known 8-bit RGBA layout so the source's own pixel
        // format (16-bit, CMYK, indexed, whatever) never matters here.
        let w = cgImage.width, h = cgImage.height
        var rgba = [UInt8](repeating: 0, count: w * h * 4)
        rgba.withUnsafeMutableBytes { buf in
            let ctx = CGContext(data: buf.baseAddress, width: w, height: h,
                                bitsPerComponent: 8, bytesPerRow: w * 4,
                                space: CGColorSpaceCreateDeviceRGB(),
                                bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue)!
            ctx.interpolationQuality = .high
            ctx.draw(cgImage, in: CGRect(x: 0, y: 0, width: w, height: h))
        }

        var out = RGBImage(width: w, height: h)
        let scale: Float = 1.0 / 255
        for i in 0..<(w * h) {
            out.r.pixels[i] = Float(rgba[i * 4]) * scale
            out.g.pixels[i] = Float(rgba[i * 4 + 1]) * scale
            out.b.pixels[i] = Float(rgba[i * 4 + 2]) * scale
        }
        return out
    }
}
