import XCTest
import simd
@testable import StitchCore

final class MatchingTests: XCTestCase {

    private func texturedImage(size: Int, seed: UInt64 = 42) -> ImageF {
        var rng = SplitMix64(seed: seed)
        var img = ImageF(width: size, height: size)
        for i in 0..<(size * size) {
            img.pixels[i] = Float(rng.next() >> 40) / Float(1 << 24) * 0.7
        }
        return Convolution.gaussianBlur(img, sigma: 1.0)
    }

    private func warp(_ img: ImageF, by h: Homography) -> ImageF {
        let hInv = h.inverse
        var out = ImageF(width: img.width, height: img.height)
        for y in 0..<img.height {
            for x in 0..<img.width {
                let p = HomographyEstimator.project(hInv, SIMD2(Double(x), Double(y)))
                out[x, y] = img.sample(x: Float(p.x), y: Float(p.y))
            }
        }
        return out
    }

    // MARK: - DLT

    func testDLTRecoversExactHomography() {
        let h = Homography(rows: [
            SIMD3(0.98, -0.10, 25.0),
            SIMD3(0.09, 1.02, -12.0),
            SIMD3(1e-5, -2e-5, 1.0),
        ])
        var from: [SIMD2<Double>] = []
        var to: [SIMD2<Double>] = []
        for (x, y) in [(10.0, 20.0), (300, 15), (280, 310), (25, 290), (150, 160), (60, 220), (240, 90), (190, 250)] {
            let p = SIMD2(x, y)
            from.append(p)
            to.append(HomographyEstimator.project(h, p))
        }
        guard let recovered = HomographyEstimator.dlt(from: from, to: to) else {
            return XCTFail("DLT returned nil")
        }
        for p in from {
            let expected = HomographyEstimator.project(h, p)
            let got = HomographyEstimator.project(recovered, p)
            XCTAssertEqual(got.x, expected.x, accuracy: 1e-6)
            XCTAssertEqual(got.y, expected.y, accuracy: 1e-6)
        }
    }

    func testDLTRejectsDegenerateInput() {
        // All points identical: no valid normalization/solution.
        let p = SIMD2<Double>(50, 50)
        XCTAssertNil(HomographyEstimator.dlt(from: [p, p, p, p], to: [p, p, p, p]))
    }

    // MARK: - Matching + RANSAC end to end

    func testMatchAndEstimateOnWarpedImage() {
        let img = texturedImage(size: 320)
        let h = Homography(rows: [
            SIMD3(cos(0.06), -sin(0.06), 18.0),
            SIMD3(sin(0.06), cos(0.06), -9.0),
            SIMD3(2e-5, 1e-5, 1.0),
        ])
        let warped = warp(img, by: h)

        let detector = SIFTDetector()
        let fa = detector.detect(in: img)
        let fb = detector.detect(in: warped)
        let matches = DescriptorMatcher.match(fa, fb)
        XCTAssertGreaterThan(matches.count, 30, "warped copy should produce many putative matches")

        guard let geometry = PairEstimator.estimate(featuresA: fa, featuresB: fb, matches: matches,
                                                    imageBWidth: warped.width, imageBHeight: warped.height) else {
            return XCTFail("no geometry estimated")
        }
        XCTAssertTrue(geometry.isVerified, "true overlap must pass verification")
        XCTAssertGreaterThan(geometry.inlierIndices.count, matches.count / 2)

        // Recovered homography should agree with ground truth away from borders.
        for (x, y) in [(80.0, 80.0), (240, 80), (160, 160), (80, 240), (240, 240)] {
            let expected = HomographyEstimator.project(h, SIMD2(x, y))
            let got = HomographyEstimator.project(geometry.homography, SIMD2(x, y))
            XCTAssertEqual(got.x, expected.x, accuracy: 2.0)
            XCTAssertEqual(got.y, expected.y, accuracy: 2.0)
        }
    }

    func testUnrelatedImagesFailVerification() {
        let a = texturedImage(size: 320, seed: 7)
        let b = texturedImage(size: 320, seed: 99)
        let detector = SIFTDetector()
        let fa = detector.detect(in: a)
        let fb = detector.detect(in: b)
        let matches = DescriptorMatcher.match(fa, fb)
        if let geometry = PairEstimator.estimate(featuresA: fa, featuresB: fb, matches: matches,
                                                 imageBWidth: b.width, imageBHeight: b.height) {
            XCTAssertFalse(geometry.isVerified, "unrelated textures must not verify as a match")
        }
        // nil geometry is also an acceptable rejection
    }

    func testMatcherRatioTestFiltersAmbiguity() {
        // Two identical descriptors in B make the best/second-best distances
        // equal, so the ratio test must reject the match.
        func feature(_ x: Float, _ desc: [Float]) -> Feature {
            var d = desc
            var norm: Float = 0
            for v in d { norm += v * v }
            norm = sqrtf(norm)
            for i in d.indices { d[i] /= norm }
            return Feature(x: x, y: 0, scale: 1, orientation: 0, response: 1, descriptor: d)
        }
        var e0 = [Float](repeating: 0.001, count: 128); e0[0] = 1
        var e1 = [Float](repeating: 0.001, count: 128); e1[1] = 1
        let a = [feature(0, e0)]
        let bAmbiguous = [feature(0, e0), feature(1, e0)]
        let bClear = [feature(0, e0), feature(1, e1)]
        XCTAssertTrue(DescriptorMatcher.match(a, bAmbiguous).isEmpty)
        XCTAssertEqual(DescriptorMatcher.match(a, bClear).count, 1)
    }
}
