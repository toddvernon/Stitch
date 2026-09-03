import CoreGraphics
import Foundation

/// Renders diagnostics (keypoint overlays, later: matches, seams) to CGImages.
public enum DebugRender {

    /// Grayscale base image with one circle per feature (radius = scale) and an
    /// orientation tick, in the style of the classic SIFT visualizations.
    public static func featureOverlay(image: ImageF, features: [Feature]) -> CGImage {
        let w = image.width, h = image.height
        var rgba = [UInt8](repeating: 0, count: w * h * 4)
        for i in 0..<(w * h) {
            let v = UInt8(min(max(image.pixels[i] * 255, 0), 255))
            rgba[i * 4] = v
            rgba[i * 4 + 1] = v
            rgba[i * 4 + 2] = v
            rgba[i * 4 + 3] = 255
        }
        return rgba.withUnsafeMutableBytes { buf -> CGImage in
            let ctx = CGContext(data: buf.baseAddress, width: w, height: h,
                                bitsPerComponent: 8, bytesPerRow: w * 4,
                                space: CGColorSpaceCreateDeviceRGB(),
                                bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue)!
            // Flip so drawing coordinates match pixel (top-left origin) coordinates.
            ctx.translateBy(x: 0, y: CGFloat(h))
            ctx.scaleBy(x: 1, y: -1)
            ctx.setLineWidth(1.0)
            ctx.setStrokeColor(CGColor(red: 1, green: 0.2, blue: 0.1, alpha: 0.9))

            for f in features {
                let r = CGFloat(max(f.scale, 1))
                let cx = CGFloat(f.x), cy = CGFloat(f.y)
                ctx.strokeEllipse(in: CGRect(x: cx - r, y: cy - r, width: 2 * r, height: 2 * r))
                ctx.move(to: CGPoint(x: cx, y: cy))
                ctx.addLine(to: CGPoint(x: cx + r * CGFloat(cosf(f.orientation)),
                                        y: cy + r * CGFloat(sinf(f.orientation))))
                ctx.strokePath()
            }
            return ctx.makeImage()!
        }
    }

    /// Side-by-side pair with correspondence lines: green for inliers,
    /// faint red for rejected putative matches.
    public static func matchOverlay(imageA: ImageF, imageB: ImageF,
                                    featuresA: [Feature], featuresB: [Feature],
                                    matches: [FeatureMatch], inlierIndices: [Int]) -> CGImage {
        let w = imageA.width + imageB.width
        let h = max(imageA.height, imageB.height)
        let xOffset = imageA.width
        var rgba = [UInt8](repeating: 0, count: w * h * 4)

        func blit(_ img: ImageF, atX: Int) {
            for y in 0..<img.height {
                for x in 0..<img.width {
                    let v = UInt8(min(max(img.pixels[y * img.width + x] * 255, 0), 255))
                    let i = (y * w + x + atX) * 4
                    rgba[i] = v; rgba[i + 1] = v; rgba[i + 2] = v; rgba[i + 3] = 255
                }
            }
        }
        blit(imageA, atX: 0)
        blit(imageB, atX: xOffset)

        let inliers = Set(inlierIndices)
        return rgba.withUnsafeMutableBytes { buf -> CGImage in
            let ctx = CGContext(data: buf.baseAddress, width: w, height: h,
                                bitsPerComponent: 8, bytesPerRow: w * 4,
                                space: CGColorSpaceCreateDeviceRGB(),
                                bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue)!
            ctx.translateBy(x: 0, y: CGFloat(h))
            ctx.scaleBy(x: 1, y: -1)
            ctx.setLineWidth(0.75)

            func drawMatch(_ k: Int, color: CGColor) {
                let m = matches[k]
                let a = featuresA[m.indexA], b = featuresB[m.indexB]
                ctx.setStrokeColor(color)
                ctx.move(to: CGPoint(x: CGFloat(a.x), y: CGFloat(a.y)))
                ctx.addLine(to: CGPoint(x: CGFloat(b.x) + CGFloat(xOffset), y: CGFloat(b.y)))
                ctx.strokePath()
            }
            let outlierColor = CGColor(red: 1, green: 0.15, blue: 0.1, alpha: 0.25)
            let inlierColor = CGColor(red: 0.1, green: 0.95, blue: 0.2, alpha: 0.8)
            for k in 0..<matches.count where !inliers.contains(k) {
                drawMatch(k, color: outlierColor)
            }
            for k in inlierIndices {
                drawMatch(k, color: inlierColor)
            }
            return ctx.makeImage()!
        }
    }
}
