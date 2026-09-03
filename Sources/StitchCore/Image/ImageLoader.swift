import Accelerate
import CoreGraphics
import Foundation
import ImageIO
import UniformTypeIdentifiers

public enum ImageLoaderError: Error {
    case cannotOpen(URL)
    case cannotDecode(URL)
    case cannotWrite(URL)
}

public enum ImageLoader {

    /// Loads an image as grayscale float [0,1], honoring EXIF orientation.
    /// If `maxDimension` is set, the image is downsampled so its longer side
    /// does not exceed it (registration runs on reduced images; rendering will
    /// re-read at full resolution).
    public static func loadGrayscale(url: URL, maxDimension: Int? = nil) throws -> ImageF {
        guard let source = CGImageSourceCreateWithURL(url as CFURL, nil) else {
            throw ImageLoaderError.cannotOpen(url)
        }
        var options: [CFString: Any] = [
            kCGImageSourceCreateThumbnailFromImageAlways: true,
            kCGImageSourceCreateThumbnailWithTransform: true,  // apply EXIF orientation
            kCGImageSourceShouldCacheImmediately: true,
        ]
        if let maxDimension {
            options[kCGImageSourceThumbnailMaxPixelSize] = maxDimension
        } else {
            // No cap: ask for full size via the image's own dimensions.
            if let props = CGImageSourceCopyPropertiesAtIndex(source, 0, nil) as? [CFString: Any],
               let w = props[kCGImagePropertyPixelWidth] as? Int,
               let h = props[kCGImagePropertyPixelHeight] as? Int {
                options[kCGImageSourceThumbnailMaxPixelSize] = max(w, h)
            }
        }
        guard let cgImage = CGImageSourceCreateThumbnailAtIndex(source, 0, options as CFDictionary)
            ?? CGImageSourceCreateImageAtIndex(source, 0, nil) else {
            throw ImageLoaderError.cannotDecode(url)
        }
        return grayscale(from: cgImage)
    }

    public static func grayscale(from cgImage: CGImage) -> ImageF {
        let w = cgImage.width, h = cgImage.height
        var bytes = [UInt8](repeating: 0, count: w * h)
        bytes.withUnsafeMutableBytes { buf in
            let ctx = CGContext(data: buf.baseAddress, width: w, height: h,
                                bitsPerComponent: 8, bytesPerRow: w,
                                space: CGColorSpaceCreateDeviceGray(),
                                bitmapInfo: CGImageAlphaInfo.none.rawValue)!
            ctx.interpolationQuality = .high
            ctx.draw(cgImage, in: CGRect(x: 0, y: 0, width: w, height: h))
        }
        var floats = [Float](repeating: 0, count: w * h)
        vDSP.convertElements(of: bytes, to: &floats)
        vDSP.divide(floats, 255, result: &floats)
        return ImageF(width: w, height: h, pixels: floats)
    }

    /// EXIF 35mm-equivalent focal length if present (for bundle-adjustment init later).
    public static func focalLength35mm(url: URL) -> Double? {
        guard let source = CGImageSourceCreateWithURL(url as CFURL, nil),
              let props = CGImageSourceCopyPropertiesAtIndex(source, 0, nil) as? [CFString: Any],
              let exif = props[kCGImagePropertyExifDictionary] as? [CFString: Any] else {
            return nil
        }
        return exif[kCGImagePropertyExifFocalLenIn35mmFilm] as? Double
    }

    public static func writePNG(_ cgImage: CGImage, to url: URL) throws {
        guard let dest = CGImageDestinationCreateWithURL(url as CFURL, UTType.png.identifier as CFString, 1, nil) else {
            throw ImageLoaderError.cannotWrite(url)
        }
        CGImageDestinationAddImage(dest, cgImage, nil)
        guard CGImageDestinationFinalize(dest) else {
            throw ImageLoaderError.cannotWrite(url)
        }
    }
}
