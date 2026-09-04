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

    /// Stored pixel dimensions without decoding the image.
    public static func pixelDimensions(url: URL) -> (width: Int, height: Int)? {
        guard let source = CGImageSourceCreateWithURL(url as CFURL, nil),
              let props = CGImageSourceCopyPropertiesAtIndex(source, 0, nil) as? [CFString: Any],
              let w = props[kCGImagePropertyPixelWidth] as? Int,
              let h = props[kCGImagePropertyPixelHeight] as? Int else {
            return nil
        }
        return (w, h)
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
        try writeImage(cgImage, to: url)
    }

    /// Writes PNG/JPEG/TIFF chosen by the destination's file extension
    /// (default PNG).
    public static func writeImage(_ cgImage: CGImage, to url: URL, jpegQuality: Double = 0.92) throws {
        let type: UTType
        switch url.pathExtension.lowercased() {
        case "jpg", "jpeg": type = .jpeg
        case "tif", "tiff": type = .tiff
        default: type = .png
        }
        guard let dest = CGImageDestinationCreateWithURL(url as CFURL, type.identifier as CFString, 1, nil) else {
            throw ImageLoaderError.cannotWrite(url)
        }
        var props: [CFString: Any] = [
            // Mark our own outputs so the pipeline never ingests a finished
            // panorama that was saved into a source folder. JPEG stores this
            // as XMP CreatorTool, PNG in its own Software field, TIFF as-is.
            kCGImagePropertyTIFFDictionary: [kCGImagePropertyTIFFSoftware: softwareTag],
            kCGImagePropertyPNGDictionary: [kCGImagePropertyPNGSoftware: softwareTag],
        ]
        if type == .jpeg {
            props[kCGImageDestinationLossyCompressionQuality] = jpegQuality
        }
        CGImageDestinationAddImage(dest, cgImage, props as CFDictionary)
        guard CGImageDestinationFinalize(dest) else {
            throw ImageLoaderError.cannotWrite(url)
        }
    }

    static let softwareTag = "Stitch"

    /// True if the file carries Stitch's own output marker, wherever the
    /// format stored it (TIFF Software, PNG Software, or JPEG XMP CreatorTool).
    public static func isStitchOutput(url: URL) -> Bool {
        guard let source = CGImageSourceCreateWithURL(url as CFURL, nil) else { return false }
        if let props = CGImageSourceCopyPropertiesAtIndex(source, 0, nil) as? [CFString: Any] {
            if let tiff = props[kCGImagePropertyTIFFDictionary] as? [CFString: Any],
               tiff[kCGImagePropertyTIFFSoftware] as? String == softwareTag {
                return true
            }
            if let png = props[kCGImagePropertyPNGDictionary] as? [CFString: Any],
               png[kCGImagePropertyPNGSoftware] as? String == softwareTag {
                return true
            }
        }
        if let meta = CGImageSourceCopyMetadataAtIndex(source, 0, nil),
           let tag = CGImageMetadataCopyTagMatchingImageProperty(
               meta, kCGImagePropertyTIFFDictionary, kCGImagePropertyTIFFSoftware),
           CGImageMetadataTagCopyValue(tag) as? String == softwareTag {
            return true
        }
        return false
    }
}
