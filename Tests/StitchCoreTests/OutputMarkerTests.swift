import CoreGraphics
import ImageIO
import UniformTypeIdentifiers
import XCTest
@testable import StitchCore

final class OutputMarkerTests: XCTestCase {

    private func tinyImage() -> CGImage {
        var rgb = RGBImage(width: 8, height: 8)
        for i in 0..<64 { rgb.r.pixels[i] = 0.5 }
        return rgb.makeCGImage()
    }

    /// Files written by Stitch are marked and excluded from input resolution,
    /// so a panorama exported into its own source folder is never re-stitched.
    func testStitchOutputsAreMarkedAndSkipped() throws {
        let dir = FileManager.default.temporaryDirectory
            .appendingPathComponent("stitch-marker-\(UUID().uuidString)")
        try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: dir) }

        // One of ours, one plain file written without the marker.
        let ours = dir.appendingPathComponent("pano.jpg")
        try ImageLoader.writeImage(tinyImage(), to: ours)
        let plain = dir.appendingPathComponent("photo.jpg")
        let dest = CGImageDestinationCreateWithURL(plain as CFURL,
                                                   UTType.jpeg.identifier as CFString, 1, nil)!
        CGImageDestinationAddImage(dest, tinyImage(), nil)
        XCTAssertTrue(CGImageDestinationFinalize(dest))

        XCTAssertTrue(ImageLoader.isStitchOutput(url: ours))
        XCTAssertFalse(ImageLoader.isStitchOutput(url: plain))

        let resolved = Stitcher.imageURLs(from: [dir])
        XCTAssertEqual(resolved.map(\.lastPathComponent), ["photo.jpg"])
    }
}
