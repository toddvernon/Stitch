import CoreGraphics
import Foundation
import StitchCore
import SwiftUI

// The app's view model: the only place the UI touches StitchCore. It owns
// the phase state machine (idle, running, done, failed), runs the pipeline
// off the main actor, and turns each StitchedPanorama into something SwiftUI
// can display cheaply while keeping the full image around for export.

/// Observable state behind `ContentView`. Main-actor isolated because every
/// published property drives the view; the pipeline itself runs detached
/// and hops back only to publish.
@MainActor
final class StitchModel: ObservableObject {

    /// The state machine `ContentView` switches on. One stitch runs at a
    /// time; `reset()` is the only way back to idle from done or failed.
    enum Phase: Equatable {
        case idle
        case running
        case done
        /// The message is shown verbatim, so it is written for the user.
        case failed(String)
    }

    /// One output ready for display and export.
    struct PanoResult: Identifiable {
        let id = UUID()
        /// The composite at output resolution; what Export writes.
        let full: CGImage
        /// Downscaled copy for the view (see `thumbnail`).
        let preview: CGImage
        /// Caption under the preview: dimensions, megapixels, span, photo count.
        let info: String
        /// Default file name in the save panel, from the source folder.
        let suggestedName: String
        /// "Panorama" or "Strip", for tab titles when there are several.
        let kindName: String
    }

    @Published var phase: Phase = .idle
    /// Progress lines from the pipeline, appended as they arrive.
    @Published var log: [String] = []
    @Published var results: [PanoResult] = []
    /// nil = auto (Pannini under 160°, spherical above).
    @Published var projection: PanoProjection? = nil
    @Published var mode: Stitcher.Mode = .auto

    /// Starts a stitch from whatever was dropped or chosen: folders, files,
    /// or a mix. Resolution to image files goes through the same rule the
    /// CLI uses. Ignored while a stitch is already running.
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

        // Only the two user-facing choices are exposed; everything else
        // stays at the Settings defaults the CLI also uses.
        var settings = Stitcher.Settings()
        settings.projection = projection
        settings.mode = mode
        // Detached so the pipeline (tens of seconds of CPU) never blocks the
        // main actor. Progress lines arrive on the worker thread and are
        // hopped back individually, which keeps the log live during the run.
        Task.detached(priority: .userInitiated) {
            do {
                let panoramas = try Stitcher.stitch(urls: urls, settings: settings) { line in
                    Task { @MainActor in
                        self.log.append(line)
                    }
                }
                // CGImage conversion and thumbnailing happen here, still off
                // the main actor, so the results view appears in one step.
                let built = panoramas.enumerated().map { (i, pano) -> PanoResult in
                    let full = pano.image.makeCGImage()
                    let mp = Double(full.width * full.height) / 1_000_000
                    let shape = pano.kind == .strip
                        ? "strip" : "\(String(format: "%.0f", pano.horizontalDegrees))° span"
                    let info = "\(full.width) × \(full.height)  (\(String(format: "%.1f", mp)) MP, "
                        + "\(shape), \(pano.sourceURLs.count) photos)"
                    // Name outputs after the folder the photos came from,
                    // numbered when a folder yields more than one.
                    let base = pano.sourceURLs.first?.deletingLastPathComponent().lastPathComponent ?? "panorama"
                    let name = panoramas.count > 1 ? "\(base)-\(i + 1)" : base
                    return PanoResult(full: full,
                                      preview: Self.thumbnail(of: full, maxWidth: 2000),
                                      info: info,
                                      suggestedName: name,
                                      kindName: pano.kind == .strip ? "Strip" : "Panorama")
                }
                await MainActor.run {
                    self.results = built
                    self.phase = built.isEmpty ? .failed("No panorama or strip could be recognized in those images.") : .done
                }
            } catch {
                await MainActor.run {
                    self.phase = .failed("\(error)")
                }
            }
        }
    }

    /// Back to the drop prompt. Drops the results, so the user is expected
    /// to have exported first; there is no undo.
    func reset() {
        phase = .idle
        log = []
        results = []
    }

    /// Downscaled copy for on-screen display. SwiftUI re-rasterizes an
    /// Image on every layout pass, and a 12000 px composite makes window
    /// resizing crawl; 2000 px is more than any window shows. `nonisolated`
    /// so it can run inside the detached task. Falls back to the original
    /// if Core Graphics cannot build the context.
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
