import CoreGraphics
import Foundation
import StitchCore
import SwiftUI

@MainActor
final class StitchModel: ObservableObject {

    enum Phase: Equatable {
        case idle
        case running
        case done
        case failed(String)
    }

    struct PanoResult: Identifiable {
        let id = UUID()
        let full: CGImage
        let preview: CGImage
        let info: String
        let suggestedName: String
    }

    @Published var phase: Phase = .idle
    @Published var log: [String] = []
    @Published var results: [PanoResult] = []

    func stitch(dropped: [URL]) {
        guard phase != .running else { return }
        let urls = Stitcher.imageURLs(from: dropped)
        guard urls.count >= 2 else {
            phase = .failed("Need at least two images (drop a folder or a selection of photos).")
            return
        }
        phase = .running
        log = ["stitching \(urls.count) images…"]
        results = []

        Task.detached(priority: .userInitiated) {
            do {
                let panoramas = try Stitcher.stitch(urls: urls) { line in
                    Task { @MainActor in
                        self.log.append(line)
                    }
                }
                let built = panoramas.enumerated().map { (i, pano) -> PanoResult in
                    let full = pano.image.makeCGImage()
                    let mp = Double(full.width * full.height) / 1_000_000
                    let info = "\(full.width) × \(full.height)  (\(String(format: "%.1f", mp)) MP, "
                        + "\(String(format: "%.0f", pano.horizontalDegrees))° span, "
                        + "\(pano.sourceURLs.count) photos)"
                    let base = pano.sourceURLs.first?.deletingLastPathComponent().lastPathComponent ?? "panorama"
                    let name = panoramas.count > 1 ? "\(base)-\(i + 1)" : base
                    return PanoResult(full: full,
                                      preview: Self.thumbnail(of: full, maxWidth: 2000),
                                      info: info,
                                      suggestedName: name)
                }
                await MainActor.run {
                    self.results = built
                    self.phase = built.isEmpty ? .failed("No panorama could be recognized in those images.") : .done
                }
            } catch {
                await MainActor.run {
                    self.phase = .failed("\(error)")
                }
            }
        }
    }

    func reset() {
        phase = .idle
        log = []
        results = []
    }

    nonisolated private static func thumbnail(of image: CGImage, maxWidth: Int) -> CGImage {
        guard image.width > maxWidth else { return image }
        let scale = Double(maxWidth) / Double(image.width)
        let w = maxWidth, h = max(1, Int(Double(image.height) * scale))
        guard let ctx = CGContext(data: nil, width: w, height: h,
                                  bitsPerComponent: 8, bytesPerRow: 0,
                                  space: CGColorSpaceCreateDeviceRGB(),
                                  bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue) else {
            return image
        }
        ctx.interpolationQuality = .high
        ctx.draw(image, in: CGRect(x: 0, y: 0, width: w, height: h))
        return ctx.makeImage() ?? image
    }
}
