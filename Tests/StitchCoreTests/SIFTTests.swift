import XCTest
@testable import StitchCore

final class SIFTTests: XCTestCase {

    // MARK: - Helpers

    /// A single Gaussian blob of the given sigma on a black background.
    private func blobImage(size: Int, cx: Float, cy: Float, sigma: Float, amplitude: Float = 0.8) -> ImageF {
        var img = ImageF(width: size, height: size)
        for y in 0..<size {
            for x in 0..<size {
                let dx = Float(x) - cx, dy = Float(y) - cy
                img[x, y] = amplitude * expf(-(dx * dx + dy * dy) / (2 * sigma * sigma))
            }
        }
        return img
    }

    /// Deterministic smooth random texture: white noise blurred a little.
    private func texturedImage(size: Int, seed: UInt64 = 42) -> ImageF {
        var state = seed
        func nextFloat() -> Float {
            // xorshift64*
            state ^= state >> 12
            state ^= state << 25
            state ^= state >> 27
            let v = state &* 2685821657736338717
            return Float(v >> 40) / Float(1 << 24)
        }
        var img = ImageF(width: size, height: size)
        for i in 0..<(size * size) {
            img.pixels[i] = nextFloat() * 0.7
        }
        return Convolution.gaussianBlur(img, sigma: 1.0)
    }

    // MARK: - Pyramid

    func testPyramidShapes() {
        let img = ImageF(width: 512, height: 256, fill: 0.5)
        let pyr = ScaleSpacePyramid(baseImage: img, config: SIFTConfig())
        XCTAssertGreaterThanOrEqual(pyr.gaussians.count, 4)
        for (o, octave) in pyr.gaussians.enumerated() {
            XCTAssertEqual(octave.count, 6)  // S + 3
            XCTAssertEqual(octave[0].width, 512 / (1 << o))
            XCTAssertEqual(octave[0].height, 256 / (1 << o))
            XCTAssertEqual(pyr.dogs[o].count, 5)
        }
    }

    func testGaussianBlurPreservesMean() {
        var img = ImageF(width: 64, height: 64, fill: 0.25)
        img[32, 32] = 1.0
        let blurred = Convolution.gaussianBlur(img, sigma: 2.0)
        let meanBefore = img.pixels.reduce(0, +) / Float(img.pixels.count)
        let meanAfter = blurred.pixels.reduce(0, +) / Float(blurred.pixels.count)
        XCTAssertEqual(meanBefore, meanAfter, accuracy: 1e-3)
    }

    // MARK: - Detection

    func testDetectsBlobAtCorrectLocationAndScale() {
        let sigma: Float = 6
        let img = blobImage(size: 200, cx: 100, cy: 100, sigma: sigma)
        let features = SIFTDetector().detect(in: img)
        XCTAssertFalse(features.isEmpty, "should detect the blob")

        // The strongest feature should sit on the blob center at a scale
        // commensurate with the blob's sigma.
        let best = features.max(by: { abs($0.response) < abs($1.response) })!
        XCTAssertEqual(best.x, 100, accuracy: 2.0)
        XCTAssertEqual(best.y, 100, accuracy: 2.0)
        XCTAssertGreaterThan(best.scale, sigma * 0.5)
        XCTAssertLessThan(best.scale, sigma * 2.0)
    }

    func testFindsFeaturesOnTexture() {
        let img = texturedImage(size: 256)
        let features = SIFTDetector().detect(in: img)
        XCTAssertGreaterThan(features.count, 50, "textured image should yield plenty of features")
        for f in features.prefix(20) {
            XCTAssertEqual(f.descriptor.count, 128)
            let norm = sqrtf(f.descriptor.reduce(0) { $0 + $1 * $1 })
            XCTAssertEqual(norm, 1.0, accuracy: 1e-3, "descriptors should be L2-normalized")
            XCTAssertTrue(f.x >= 0 && f.x < 256 && f.y >= 0 && f.y < 256)
        }
    }

    /// Gradients are invariant to a constant brightness offset, so the detector
    /// must produce identical features (SIFT's illumination invariance).
    func testBrightnessOffsetInvariance() {
        let img = texturedImage(size: 256)
        var brighter = img
        for i in brighter.pixels.indices {
            brighter.pixels[i] += 0.2  // values stay < 1, no clipping
        }
        let a = SIFTDetector().detect(in: img)
        let b = SIFTDetector().detect(in: brighter)
        XCTAssertFalse(a.isEmpty)
        XCTAssertEqual(a.count, b.count)
        func sortedPositions(_ features: [Feature]) -> [SIMD2<Float>] {
            let pos: [SIMD2<Float>] = features.map { SIMD2($0.x, $0.y) }
            return pos.sorted { (l: SIMD2<Float>, r: SIMD2<Float>) -> Bool in
                if l.x != r.x { return l.x < r.x }
                return l.y < r.y
            }
        }
        for (pa, pb) in zip(sortedPositions(a), sortedPositions(b)) {
            XCTAssertEqual(pa.x, pb.x, accuracy: 0.01)
            XCTAssertEqual(pa.y, pb.y, accuracy: 0.01)
        }
    }

    /// Rotating the image by 90° should yield features at rotated positions:
    /// the core of SIFT's rotation invariance (exact for a lossless rotation).
    func testRotation90FeatureCount() {
        let img = texturedImage(size: 256)
        var rotated = ImageF(width: 256, height: 256)
        for y in 0..<256 {
            for x in 0..<256 {
                rotated[255 - y, x] = img[x, y]
            }
        }
        let a = SIFTDetector().detect(in: img)
        let b = SIFTDetector().detect(in: rotated)
        XCTAssertFalse(a.isEmpty)
        XCTAssertEqual(Float(a.count), Float(b.count), accuracy: max(Float(a.count) * 0.05, 1),
                       "feature count should be stable under 90° rotation")
    }
}
