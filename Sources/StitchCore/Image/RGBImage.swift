import Accelerate
import CoreGraphics
import Foundation
import ImageIO

/// Planar float RGB image, values nominally [0, 1].
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

    public func sample(x: Float, y: Float) -> SIMD3<Float> {
        SIMD3(r.sample(x: x, y: y), g.sample(x: x, y: y), b.sample(x: x, y: y))
    }

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

    public func makeCGImage() -> CGImage {
        let w = width, h = height
        var rgba = [UInt8](repeating: 255, count: w * h * 4)
        for i in 0..<(w * h) {
            rgba[i * 4] = UInt8(min(max(r.pixels[i] * 255, 0), 255))
            rgba[i * 4 + 1] = UInt8(min(max(g.pixels[i] * 255, 0), 255))
            rgba[i * 4 + 2] = UInt8(min(max(b.pixels[i] * 255, 0), 255))
        }
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
            options[kCGImageSourceThumbnailMaxPixelSize] = max(w, h)
        }
        guard let cgImage = CGImageSourceCreateThumbnailAtIndex(source, 0, options as CFDictionary)
            ?? CGImageSourceCreateImageAtIndex(source, 0, nil) else {
            throw ImageLoaderError.cannotDecode(url)
        }

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
