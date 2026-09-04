import XCTest
import simd
@testable import StitchCore

final class StripTests: XCTestCase {

    // MARK: - Similarity

    func testSimilarityFitRecoversExactTransform() {
        let s = Similarity(scale: 1.07, rotation: 0.05, translation: SIMD2(310, -12))
        let from: [SIMD2<Double>] = [SIMD2(10, 20), SIMD2(400, 30), SIMD2(380, 290), SIMD2(25, 300), SIMD2(200, 160)]
        let to = from.map { s.apply($0) }
        let fit = Similarity.fit(from: from, to: to)!
        XCTAssertEqual(fit.a, s.a, accuracy: 1e-9)
        XCTAssertEqual(fit.b, s.b, accuracy: 1e-9)
        XCTAssertEqual(fit.tx, s.tx, accuracy: 1e-7)
        XCTAssertEqual(fit.ty, s.ty, accuracy: 1e-7)

        // Two points determine it exactly.
        let two = Similarity.fit(from: Array(from[0...1]), to: Array(to[0...1]))!
        XCTAssertEqual(two.a, s.a, accuracy: 1e-9)
        XCTAssertEqual(two.tx, s.tx, accuracy: 1e-7)
    }

    func testSimilarityInverseAndComposeAndHomographyRoundTrip() {
        let s = Similarity(scale: 0.9, rotation: -0.2, translation: SIMD2(-40, 15))
        let p = SIMD2(123.0, 45.0)
        let back = s.inverse.apply(s.apply(p))
        XCTAssertEqual(back.x, p.x, accuracy: 1e-9)
        XCTAssertEqual(back.y, p.y, accuracy: 1e-9)

        let t = Similarity(scale: 1.3, rotation: 0.4, translation: SIMD2(5, 5))
        let composed = t.composed(with: s).apply(p)
        let sequential = t.apply(s.apply(p))
        XCTAssertEqual(composed.x, sequential.x, accuracy: 1e-9)
        XCTAssertEqual(composed.y, sequential.y, accuracy: 1e-9)

        let viaH = HomographyEstimator.project(s.homography, p)
        XCTAssertEqual(viaH.x, s.apply(p).x, accuracy: 1e-9)
        XCTAssertEqual(viaH.y, s.apply(p).y, accuracy: 1e-9)
        let parsed = Similarity(homography: s.homography * 2)!  // scale-invariant
        XCTAssertEqual(parsed, s)
    }

    // MARK: - Similarity RANSAC

    /// Sparse true matches drowned in outliers: the case that defeats a
    /// four-point homography and that two-point similarity RANSAC handles.
    func testSimilarityRANSACFindsSparseInliersAmongOutliers() {
        let truth = Similarity(scale: 1.02, rotation: 0.02, translation: SIMD2(-600, 8))
        var rng = SplitMix64(seed: 7)
        func rand(_ range: ClosedRange<Double>) -> Double {
            range.lowerBound + (range.upperBound - range.lowerBound) * Double(rng.next() >> 11) / Double(1 << 53)
        }
        var featuresA: [Feature] = []
        var featuresB: [Feature] = []
        var matches: [FeatureMatch] = []
        // 12 true matches.
        for _ in 0..<12 {
            let p = SIMD2(rand(700...1900), rand(500...900))
            let q = truth.apply(p)
            featuresA.append(Feature(x: Float(p.x), y: Float(p.y), scale: 2, orientation: 0, response: 1, descriptor: []))
            featuresB.append(Feature(x: Float(q.x), y: Float(q.y), scale: 2, orientation: 0, response: 1, descriptor: []))
            matches.append(FeatureMatch(indexA: featuresA.count - 1, indexB: featuresB.count - 1, distance: 0.1))
        }
        // 110 random junk matches.
        for _ in 0..<110 {
            featuresA.append(Feature(x: Float(rand(0...2000)), y: Float(rand(0...1500)), scale: 2, orientation: 0, response: 1, descriptor: []))
            featuresB.append(Feature(x: Float(rand(0...2000)), y: Float(rand(0...1500)), scale: 2, orientation: 0, response: 1, descriptor: []))
            matches.append(FeatureMatch(indexA: featuresA.count - 1, indexB: featuresB.count - 1, distance: 0.1))
        }

        let strip = PairEstimator.estimate(featuresA: featuresA, featuresB: featuresB, matches: matches,
                                           imageBWidth: 2000, imageBHeight: 1500, model: .similarity)!
        XCTAssertTrue(strip.isVerified)
        XCTAssertEqual(strip.inlierIndices.count, 12)
        let s = strip.similarity!
        XCTAssertEqual(s.scale, truth.scale, accuracy: 1e-3)
        XCTAssertEqual(s.tx, truth.tx, accuracy: 1.0)

        // The rotational model with the same data does not verify.
        let pano = PairEstimator.estimate(featuresA: featuresA, featuresB: featuresB, matches: matches,
                                          imageBWidth: 2000, imageBHeight: 1500, model: .homography)
        XCTAssertFalse(pano?.isVerified ?? false)
    }

    // MARK: - Strip aligner

    /// Four images along a facade with a bit of tilt per shot, connected by
    /// adjacent pairs and one skip pair: the global solve must recover the
    /// relative placements up to the gauge (mean rotation 0, mean scale 1).
    func testStripAlignerRecoversChain() {
        let truth = [
            Similarity(scale: 1.00, rotation: 0.010, translation: SIMD2(0, 0)),
            Similarity(scale: 1.02, rotation: -0.020, translation: SIMD2(700, 10)),
            Similarity(scale: 0.98, rotation: 0.015, translation: SIMD2(1380, -6)),
            Similarity(scale: 1.00, rotation: -0.005, translation: SIMD2(2100, 4)),
        ]
        var rng = SplitMix64(seed: 11)
        func rand(_ range: ClosedRange<Double>) -> Double {
            range.lowerBound + (range.upperBound - range.lowerBound) * Double(rng.next() >> 11) / Double(1 << 53)
        }
        var features = [[Feature]](repeating: [], count: 4)
        var pairs: [VerifiedPair] = []
        for (i, j) in [(0, 1), (1, 2), (2, 3), (0, 2)] {
            var matches: [FeatureMatch] = []
            for _ in 0..<15 {
                // A strip-frame point seen by both images.
                // Images are 2000 wide, so i and j both see x ∈ [700·max, 700·min + 2000).
                let world = SIMD2(rand(Double(max(i, j)) * 700 + 50 ... Double(min(i, j)) * 700 + 1950),
                                  rand(100...900))
                let p = truth[i].inverse.apply(world) + SIMD2(rand(-0.3...0.3), rand(-0.3...0.3))
                let q = truth[j].inverse.apply(world) + SIMD2(rand(-0.3...0.3), rand(-0.3...0.3))
                features[i].append(Feature(x: Float(p.x), y: Float(p.y), scale: 2, orientation: 0, response: 1, descriptor: []))
                features[j].append(Feature(x: Float(q.x), y: Float(q.y), scale: 2, orientation: 0, response: 1, descriptor: []))
                matches.append(FeatureMatch(indexA: features[i].count - 1, indexB: features[j].count - 1, distance: 0))
            }
            let geometry = PairGeometry(homography: matrix_identity_double3x3,
                                        inlierIndices: Array(matches.indices),
                                        overlapMatchCount: matches.count, isVerified: true)
            pairs.append(VerifiedPair(indexA: i, indexB: j, matches: matches, geometry: geometry))
        }
        let group = PanoramaGroup(imageIndices: [0, 1, 2, 3], pairs: pairs)
        let alignment = StripAligner.align(group: group, features: features)!
        XCTAssertLessThan(alignment.finalRMS, 1.0)

        // Gauge conditions.
        let solved = (0..<4).map { alignment.transforms[$0]! }
        XCTAssertEqual(solved.map(\.rotation).reduce(0, +) / 4, 0, accuracy: 1e-6)
        XCTAssertEqual(solved.map { log($0.scale) }.reduce(0, +) / 4, 0, accuracy: 1e-6)

        // Relative transforms match the truth: T_j^-1 ∘ T_i is gauge-free.
        for i in 0..<4 {
            for j in 0..<4 where i != j {
                let want = truth[j].inverse.composed(with: truth[i])
                let got = solved[j].inverse.composed(with: solved[i])
                for p in [SIMD2(0.0, 0.0), SIMD2(2000.0, 1500.0), SIMD2(500.0, 1200.0)] {
                    let a = want.apply(p), b = got.apply(p)
                    XCTAssertEqual(a.x, b.x, accuracy: 1.5, "\(i)->\(j)")
                    XCTAssertEqual(a.y, b.y, accuracy: 1.5, "\(i)->\(j)")
                }
            }
        }
    }

    // MARK: - Strip geometry

    func testStripGeometryProjectsImagesToTheirPlacement() {
        let transforms = [0: Similarity.identity,
                          1: Similarity(scale: 1, rotation: 0, translation: SIMD2(150, 20))]
        let sizes = [0: (width: 200, height: 100), 1: (width: 200, height: 100)]
        let geo = StripGeometry(transforms: transforms, sizes: sizes, outputWidth: 350)!
        XCTAssertEqual(geo.width, 350)
        XCTAssertEqual(geo.height, 120)
        XCTAssertEqual(geo.scale, 1, accuracy: 1e-12)

        // Output pixel ↔ strip point round trip.
        let p = geo.stripPoint(px: 40, py: 30)
        let back = geo.outputPoint(p)
        XCTAssertEqual(back.x, 40, accuracy: 1e-9)
        XCTAssertEqual(back.y, 30, accuracy: 1e-9)

        var flat = RGBImage(width: 200, height: 100)
        for i in flat.r.pixels.indices { flat.r.pixels[i] = 0.5 }
        let layer = geo.project(imageIndex: 1, image: flat)!
        // Covers roughly x ∈ [150, 350), y ∈ [20, 120) with 2 px padding.
        XCTAssertLessThanOrEqual(layer.x0, 150)
        XCTAssertGreaterThanOrEqual(layer.x0 + layer.width, 348)
        XCTAssertEqual(layer.validity[160 - layer.x0, 60 - layer.y0], 1)
        XCTAssertEqual(layer.validity[max(0, 140 - layer.x0), 60 - layer.y0], 0)
        XCTAssertEqual(layer.rgb.r[160 - layer.x0, 60 - layer.y0], 0.5, accuracy: 1e-6)
        // Tent peaks at the image center.
        let center = layer.tent[250 - layer.x0, 70 - layer.y0]
        let edge = layer.tent[160 - layer.x0, 60 - layer.y0]
        XCTAssertGreaterThan(center, edge)

        let half = geo.scaled(toWidth: 175)
        XCTAssertEqual(half.height, 60)
        XCTAssertEqual(half.sourceDimension(for: 1), 120)  // 200 × 0.5 × 1.2
    }

    // MARK: - Seam locality

    /// Two identical flat layers overlap: the plain graph cut has no
    /// preference, the locality term puts the seam at the midpoint where
    /// the two tents cross.
    func testSeamLocalityPrefersNearerImageCenter() {
        func layer(index: Int, x0: Int) -> ImageLayer {
            let w = 100, h = 20
            var rgb = RGBImage(width: w, height: h)
            for i in rgb.r.pixels.indices { rgb.r.pixels[i] = 0.4; rgb.g.pixels[i] = 0.4; rgb.b.pixels[i] = 0.4 }
            var tent = ImageF(width: w, height: h)
            for y in 0..<h {
                for x in 0..<w {
                    tent[x, y] = Float(1 - abs(2 * (Double(x) + 0.5) / Double(w) - 1))
                }
            }
            return ImageLayer(imageIndex: index, x0: x0, y0: 0, rgb: rgb,
                              validity: ImageF(width: w, height: h, fill: 1), tent: tent)
        }
        let a = layer(index: 0, x0: 0)
        let b = layer(index: 1, x0: 60)
        let labels = SeamFinder.labels(layers: [a, b], width: 160, height: 20, localityWeight: 0.01)
        // Overlap is x ∈ [60, 100); tents cross at x = 80.
        for y in 0..<20 {
            XCTAssertEqual(labels[y * 160 + 70], 0, "left of midpoint belongs to image 0")
            XCTAssertEqual(labels[y * 160 + 90], 1, "right of midpoint belongs to image 1")
        }
    }
}
