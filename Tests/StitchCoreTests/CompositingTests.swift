import XCTest
@testable import StitchCore

final class CompositingTests: XCTestCase {

    private func constantLayer(index: Int, x0: Int, y0: Int, w: Int, h: Int,
                               value: Float) -> ImageLayer {
        var rgb = RGBImage(width: w, height: h)
        for i in 0..<(w * h) {
            rgb.r.pixels[i] = value
            rgb.g.pixels[i] = value
            rgb.b.pixels[i] = value
        }
        return ImageLayer(imageIndex: index, x0: x0, y0: y0, rgb: rgb,
                          validity: ImageF(width: w, height: h, fill: 1),
                          tent: ImageF(width: w, height: h, fill: 1))
    }

    // MARK: - Max flow

    func testMaxFlowKnownGraph() {
        // s→0 (3), s→1 (2), 0→1 (1), 0→t (2), 1→t (3):
        // 2 via 0→t, 1 via 0→1→t, 2 via 1→t = 5.
        let f = MaxFlow(nodeCount: 2)
        f.addSourceEdge(0, cap: 3)
        f.addSourceEdge(1, cap: 2)
        f.addEdge(0, 1, cap: 1)
        f.addSinkEdge(0, cap: 2)
        f.addSinkEdge(1, cap: 3)
        XCTAssertEqual(f.solve(), 5, accuracy: 1e-9)
    }

    func testMaxFlowCutSides() {
        // Bottleneck between 0 and 1: cut separates them.
        let f = MaxFlow(nodeCount: 2)
        f.addSourceEdge(0, cap: 100)
        f.addEdge(0, 1, cap: 1, capRev: 1)
        f.addSinkEdge(1, cap: 100)
        XCTAssertEqual(f.solve(), 1, accuracy: 1e-9)
        XCTAssertTrue(f.isSourceSide(0))
        XCTAssertFalse(f.isSourceSide(1))
    }

    // MARK: - Gain

    func testGainCompensationEqualizesExposure() {
        // Same scene, image 1 photographed 25% brighter, half-overlapping.
        let a = constantLayer(index: 0, x0: 0, y0: 0, w: 100, h: 60, value: 0.40)
        let b = constantLayer(index: 1, x0: 50, y0: 0, w: 100, h: 60, value: 0.50)
        let gains = GainCompensator.solve(layers: [a, b])
        let ga = gains[0]!, gb = gains[1]!
        // With the paper's σ_N/σ_g the prior intentionally stops short of full
        // equalization (multi-band blending absorbs the remainder, §7); require
        // the mismatch to shrink to under a third of the uncompensated 0.10...
        let compensated = abs(ga * 0.40 - gb * 0.50)
        XCTAssertLessThan(compensated, 0.033)
        XCTAssertGreaterThan(ga, gb, "the darker image gains up relative to the brighter one")
        // ...while the prior keeps the mean gain near 1.
        XCTAssertEqual((ga + gb) / 2, 1.0, accuracy: 0.1)
    }

    func testGainMapBilinear() {
        let m = GainMap(cols: 2, rows: 1, values: [1.0, 2.0])
        // Block centers hold their values; halfway between them interpolates;
        // beyond the outer centers the edge value holds.
        XCTAssertEqual(m.gain(u: 0.25, v: 0.5), 1.0, accuracy: 1e-12)
        XCTAssertEqual(m.gain(u: 0.75, v: 0.5), 2.0, accuracy: 1e-12)
        XCTAssertEqual(m.gain(u: 0.5, v: 0.5), 1.5, accuracy: 1e-12)
        XCTAssertEqual(m.gain(u: 0.0, v: 0.5), 1.0, accuracy: 1e-12)
        XCTAssertEqual(m.gain(u: 1.0, v: 0.5), 2.0, accuracy: 1e-12)
        XCTAssertEqual(GainMap(constant: 0.8).gain(u: 0.3, v: 0.9), 0.8, accuracy: 1e-12)
    }

    func testBlockGainMatchesSingleGainOnUniformExposure() {
        // Same setup as the single-gain test: block gains should come out
        // flat across each image and equalize about as far.
        let a = constantLayer(index: 0, x0: 0, y0: 0, w: 100, h: 60, value: 0.40)
        let b = constantLayer(index: 1, x0: 50, y0: 0, w: 100, h: 60, value: 0.50)
        let maps = GainCompensator.solveBlocks(layers: [a, b], blocksAcross: 5, minBlockSize: 10)
        let ma = maps[0]!, mb = maps[1]!
        XCTAssertLessThan(ma.values.max()! - ma.values.min()!, 0.02, "uniform image gets a flat map")
        XCTAssertLessThan(mb.values.max()! - mb.values.min()!, 0.02)
        XCTAssertLessThan(abs(ma.mean * 0.40 - mb.mean * 0.50), 0.033)
        XCTAssertGreaterThan(ma.mean, mb.mean)
        XCTAssertEqual((ma.mean + mb.mean) / 2, 1.0, accuracy: 0.1)
    }

    func testBlockGainCorrectsSpatiallyVaryingExposure() {
        // B is 25% brighter than A in its top half only (a sky that the phone
        // exposed differently) and identical in the bottom half. One gain per
        // image has to compromise; block gains should fix the top without
        // disturbing the bottom.
        let a = constantLayer(index: 0, x0: 0, y0: 0, w: 100, h: 60, value: 0.40)
        var b = constantLayer(index: 1, x0: 50, y0: 0, w: 100, h: 60, value: 0.40)
        for y in 0..<30 {
            for x in 0..<100 {
                b.rgb.r.pixels[y * 100 + x] = 0.50
                b.rgb.g.pixels[y * 100 + x] = 0.50
                b.rgb.b.pixels[y * 100 + x] = 0.50
            }
        }
        // Mismatch in the overlap (pano x 50..<100) for a pair of corrected
        // layers, over a band of rows.
        func mismatch(_ la: ImageLayer, _ lb: ImageLayer, rows: Range<Int>) -> Float {
            var worst: Float = 0
            for y in rows {
                for x in 50..<100 {
                    worst = max(worst, abs(la.rgb.r.pixels[y * 100 + x] - lb.rgb.r.pixels[y * 100 + (x - 50)]))
                }
            }
            return worst
        }
        // Single gain: one compromise for both halves.
        var sa = a, sb = b
        let single = GainCompensator.solve(layers: [a, b])
        GainCompensator.apply(gain: single[0]!, to: &sa)
        GainCompensator.apply(gain: single[1]!, to: &sb)
        // Block gain: 5×3 blocks over each 100×60 layer, so the top and
        // bottom block rows are centered at y = 10 and y = 50.
        var la = a, lb = b
        let maps = GainCompensator.solveBlocks(layers: [a, b], blocksAcross: 5, minBlockSize: 10)
        GainCompensator.apply(map: maps[0]!, to: &la)
        GainCompensator.apply(map: maps[1]!, to: &lb)
        let top = 4..<16, bottom = 44..<56
        XCTAssertLessThan(mismatch(la, lb, rows: top), mismatch(sa, sb, rows: top),
                          "block gain beats the single gain where exposure differs")
        XCTAssertLessThan(mismatch(la, lb, rows: bottom), mismatch(sa, sb, rows: bottom),
                          "and where it does not")
        // Absolute: the 0.10 top gap shrinks by about half. The paper's
        // prior and the block smoothness both stop short of full
        // equalization (the single-gain test documents the same), which is
        // exactly what the Hilton Head sky showed. The matched bottom is
        // not pushed apart.
        XCTAssertLessThan(mismatch(la, lb, rows: top), 0.05)
        XCTAssertLessThan(mismatch(la, lb, rows: bottom), 0.02)
        // And the map itself carries the structure: B's top blocks gain
        // down relative to its bottom blocks.
        let mb = maps[1]!
        let topRow = Array(mb.values[0..<mb.cols]), bottomRow = Array(mb.values[(mb.rows - 1) * mb.cols..<mb.values.count])
        XCTAssertLessThan(topRow.reduce(0, +) / Double(mb.cols), bottomRow.reduce(0, +) / Double(mb.cols))
    }

    // MARK: - Seams

    /// Two half-overlapping images identical in the overlap except a bright
    /// square present only in B: the seam must not cut through the square,
    /// so its pixels must all carry the same label.
    func testSeamAvoidsMovingObject() {
        var a = constantLayer(index: 0, x0: 0, y0: 0, w: 120, h: 80, value: 0.5)
        var b = constantLayer(index: 1, x0: 60, y0: 0, w: 120, h: 80, value: 0.5)
        // Shared pano-space texture so both images agree in the overlap.
        for y in 0..<80 {
            for x in 0..<120 {
                let ta = Float((x + y) % 7) * 0.01          // pano x = x for A
                a.rgb.r.pixels[y * 120 + x] += ta
                a.rgb.g.pixels[y * 120 + x] += ta
                a.rgb.b.pixels[y * 120 + x] += ta
                let tb = Float((60 + x + y) % 7) * 0.01     // pano x = 60 + x for B
                b.rgb.r.pixels[y * 120 + x] += tb
                b.rgb.g.pixels[y * 120 + x] += tb
                b.rgb.b.pixels[y * 120 + x] += tb
            }
        }
        // "Moving object": bright square only in B, inside the overlap
        // (pano x 80..95, y 30..45 → B-local x 20..35).
        for y in 30..<46 {
            for x in 20..<36 {
                b.rgb.r.pixels[y * 120 + x] = 1.0
                b.rgb.g.pixels[y * 120 + x] = 1.0
                b.rgb.b.pixels[y * 120 + x] = 1.0
            }
        }

        let labels = SeamFinder.labels(layers: [a, b], width: 180, height: 80)

        // Every pixel of the square region must have a single label.
        var seen = Set<Int32>()
        for y in 30..<46 {
            for x in 80..<96 {
                seen.insert(labels[y * 180 + x])
            }
        }
        XCTAssertEqual(seen.count, 1, "seam must not cut through the differing object")
        // And every covered pixel is labeled.
        for y in 0..<80 {
            for x in 0..<180 {
                XCTAssertGreaterThanOrEqual(labels[y * 180 + x], 0)
            }
        }
    }

    // MARK: - Multi-band blending

    func testMultiBandBlendSmoothTransition() {
        // Red image left, blue image right, overlapping strip; seam at center.
        let w = 200, h = 64
        var red = RGBImage(width: 120, height: h)
        var blue = RGBImage(width: 120, height: h)
        for i in 0..<(120 * h) {
            red.r.pixels[i] = 0.9
            red.g.pixels[i] = 0.1
            red.b.pixels[i] = 0.1
            blue.r.pixels[i] = 0.1
            blue.g.pixels[i] = 0.1
            blue.b.pixels[i] = 0.9
        }
        let valid = ImageF(width: 120, height: h, fill: 1)
        var seamRed = ImageF(width: 120, height: h)
        var seamBlue = ImageF(width: 120, height: h)
        for y in 0..<h {
            for x in 0..<120 {
                // Red owns pano x < 100; blue owns pano x >= 100.
                seamRed[x, y] = x < 100 ? 1 : 0          // red at x0=0
                seamBlue[x, y] = (80 + x) >= 100 ? 1 : 0 // blue at x0=80
            }
        }

        let blender = MultiBandBlender(width: w, height: h, levels: 4)
        blender.add(rgb: red, validity: valid, seamMask: seamRed, x0: 0, y0: 0)
        blender.add(rgb: blue, validity: valid, seamMask: seamBlue, x0: 80, y0: 0)
        let out = blender.finalize()

        let y = h / 2
        // Far sides keep their colors.
        XCTAssertEqual(out.r.pixels[y * w + 10], 0.9, accuracy: 0.05)
        XCTAssertEqual(out.b.pixels[y * w + 10], 0.1, accuracy: 0.05)
        XCTAssertEqual(out.r.pixels[y * w + 190], 0.1, accuracy: 0.05)
        XCTAssertEqual(out.b.pixels[y * w + 190], 0.9, accuracy: 0.05)
        // At the seam the blend is intermediate, transitioning monotonically.
        let atSeam = out.r.pixels[y * w + 100]
        XCTAssertGreaterThan(atSeam, 0.2)
        XCTAssertLessThan(atSeam, 0.8)
        // No NaNs anywhere.
        for v in out.r.pixels + out.g.pixels + out.b.pixels {
            XCTAssertFalse(v.isNaN)
        }
    }

    // MARK: - Crop

    func testLargestCoveredRect() {
        // Coverage: full 100x60 rectangle except a notch cut from the top-left.
        var mask = ImageF(width: 100, height: 60, fill: 1)
        for y in 0..<20 {
            for x in 0..<30 {
                mask[x, y] = 0
            }
        }
        let rect = Compositor.largestCoveredRect(coverage: mask)
        // Best is the 100x40 band below the notch (area 4000) vs 70x60 (4200).
        XCTAssertEqual(rect.w * rect.h, 4200)
        XCTAssertEqual(rect.x, 30)
        XCTAssertEqual(rect.y, 0)
    }
}
