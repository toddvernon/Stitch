// Compositing stage 8. Runs on the low-resolution layers before seam
// finding, so the seam cost sees exposure-matched images; the solved gains
// are then applied again to the full-resolution layers before blending.

import Foundation

/// A per-image gain that varies across the frame: a coarse grid of gains
/// over the layer's bounding box, bilinearly interpolated at apply time.
/// A 1×1 map is a plain scalar gain. Grid coordinates are normalized to the
/// bounding box, so a map solved on the low-res layer applies unchanged to
/// the full-res layer of the same image.
public struct GainMap {
    public var cols: Int
    public var rows: Int
    /// Row-major block gains; block (c, r) is centered at ((c+0.5)/cols, (r+0.5)/rows).
    public var values: [Double]

    public init(cols: Int, rows: Int, values: [Double]) {
        precondition(values.count == cols * rows)
        self.cols = cols
        self.rows = rows
        self.values = values
    }

    /// A uniform gain.
    public init(constant: Double) {
        self.init(cols: 1, rows: 1, values: [constant])
    }

    /// Average block gain, for logging.
    public var mean: Double { values.reduce(0, +) / Double(values.count) }

    /// Bilinear gain at normalized position (u, v) in [0, 1]². Outside the
    /// outer block centers the map holds its edge value.
    public func gain(u: Double, v: Double) -> Double {
        let (c0, c1, wx) = GainMap.span(u, count: cols)
        let (r0, r1, wy) = GainMap.span(v, count: rows)
        let top = values[r0 * cols + c0] * (1 - wx) + values[r0 * cols + c1] * wx
        let bottom = values[r1 * cols + c0] * (1 - wx) + values[r1 * cols + c1] * wx
        return top * (1 - wy) + bottom * wy
    }

    /// Neighboring block indices and the interpolation weight for one axis.
    static func span(_ t: Double, count: Int) -> (Int, Int, Double) {
        guard count > 1 else { return (0, 0, 0) }
        let f = min(max(t * Double(count) - 0.5, 0), Double(count - 1))
        let i0 = min(Int(f), count - 2)
        return (i0, i0 + 1, f - Double(i0))
    }
}

/// Closed-form gain compensation (Brown & Lowe IJCV 2007 §6): gains that
/// minimize normalized intensity error over all pairwise overlaps, with a
/// prior keeping gains near 1. σ_N = 10/255, σ_g = 0.1 as in the paper.
///
/// Two solvers share that model. `solve` is the paper's single gain per
/// image. `solveBlocks` is the block-based variant DESIGN.md calls for: the
/// same least-squares fit over a coarse grid of blocks per image, tied
/// together by a smoothness term, so the sky and the foliage of one photo
/// can receive different corrections. One scalar per image cannot do that
/// when a phone's auto exposure and local tone mapping have shifted the sky
/// by a different amount than the ground (the Hilton Head sky band).
public enum GainCompensator {

    /// Normalized intensity error scale, in [0, 1] units.
    static let sigmaN = 10.0 / 255.0
    /// Gain prior scale: how far from 1 a gain may drift cheaply.
    static let sigmaG = 0.1

    /// Gain per image index. Images with no usable overlap keep 1, and every
    /// gain is clamped to [0.5, 2] so a bad overlap statistic (sky-only
    /// overlap, a moving object) can neither blow out nor black out a photo.
    public static func solve(layers: [ImageLayer]) -> [Int: Double] {
        let n = layers.count
        guard n >= 2 else { return Dictionary(uniqueKeysWithValues: layers.map { ($0.imageIndex, 1.0) }) }

        // Overlap statistics: pixel count and mean intensities per pair.
        var count = [Double](repeating: 0, count: n * n)
        var meanSelf = [Double](repeating: 0, count: n * n)   // Ī_ij: mean of i over i∩j

        for a in 0..<n {
            for b in (a + 1)..<n {
                let la = layers[a], lb = layers[b]
                let x0 = max(la.x0, lb.x0), x1 = min(la.x0 + la.width, lb.x0 + lb.width)
                let y0 = max(la.y0, lb.y0), y1 = min(la.y0 + la.height, lb.y0 + lb.height)
                guard x1 > x0, y1 > y0 else { continue }
                var nPix = 0.0
                var sumA = 0.0, sumB = 0.0
                for y in y0..<y1 {
                    for x in x0..<x1 {
                        let ia = (y - la.y0) * la.width + (x - la.x0)
                        let ib = (y - lb.y0) * lb.width + (x - lb.x0)
                        guard la.validity.pixels[ia] > 0.5, lb.validity.pixels[ib] > 0.5 else { continue }
                        nPix += 1
                        sumA += intensity(la, ia)
                        sumB += intensity(lb, ib)
                    }
                }
                // A handful of overlap pixels gives a meaningless mean;
                // treat the pair as not overlapping.
                guard nPix > 25 else { continue }
                count[a * n + b] = nPix
                count[b * n + a] = nPix
                meanSelf[a * n + b] = sumA / nPix
                meanSelf[b * n + a] = sumB / nPix
            }
        }

        // Normal equations of the paper's eq. 29:
        //   e = Σ_ij N_ij [ (g_i·Ī_ij − g_j·Ī_ji)² / σ_N² + (1 − g_i)² / σ_g² ]
        // is quadratic in the gains, so ∂e/∂g_i = 0 is one linear row per
        // image: the diagonal collects both the data and the prior term, the
        // off-diagonal couples i to each j it overlaps, and the right-hand
        // side is the prior pulling toward 1.
        var a = [Double](repeating: 0, count: n * n)
        var b = [Double](repeating: 0, count: n)
        for i in 0..<n {
            for j in 0..<n where j != i {
                let nij = count[i * n + j]
                guard nij > 0 else { continue }
                let iij = meanSelf[i * n + j]
                let iji = meanSelf[j * n + i]
                a[i * n + i] += nij * (iij * iij / (sigmaN * sigmaN) + 1 / (sigmaG * sigmaG))
                a[i * n + j] -= nij * iij * iji / (sigmaN * sigmaN)
                b[i] += nij / (sigmaG * sigmaG)
            }
        }

        // Images with no overlap statistics keep gain 1.
        for i in 0..<n where a[i * n + i] == 0 {
            a[i * n + i] = 1
            b[i] = 1
        }

        // A is symmetric positive definite (the prior term guarantees it), so
        // the bundle adjuster's Cholesky solver applies as is.
        guard let g = BundleAdjuster.choleskySolve(a, b, n: n) else {
            return Dictionary(uniqueKeysWithValues: layers.map { ($0.imageIndex, 1.0) })
        }
        var result: [Int: Double] = [:]
        for (k, layer) in layers.enumerated() {
            result[layer.imageIndex] = min(max(g[k], 0.5), 2.0)
        }
        return result
    }

    /// Block-based gains: one `GainMap` per image index.
    ///
    /// Every layer is cut into a grid of roughly `blocksAcross` blocks along
    /// its longer side (never smaller than `minBlockSize` pixels, so each
    /// block has a meaningful mean). Each block is an unknown gain. Two
    /// blocks of different images that overlap get the paper's data and
    /// prior terms over their shared pixels, exactly as `solve` does for
    /// whole images; 4-neighbor blocks within one image get a smoothness
    /// term `smoothness · (g_p − g_q)²`, scaled to the prior's per-pixel
    /// weight so the two are comparable. Blocks with no overlap at all have
    /// only smoothness rows, which makes their gain the harmonic
    /// interpolation of the overlapped blocks around them rather than a
    /// pull back toward 1 that would put a gradient across the frame.
    ///
    /// The coarse grid is deliberate. It separates sky from ground and the
    /// vignetted corners from the center, which is what a single gain
    /// misses, while keeping the dense solve small (tens of blocks per
    /// image) and leaving nothing fine enough to fit moving objects.
    public static func solveBlocks(layers: [ImageLayer],
                                   blocksAcross: Int = 10,
                                   minBlockSize: Int = 16,
                                   smoothness: Double = 1.0,
                                   priorSigma: Double = 0.1) -> [Int: GainMap] {
        let n = layers.count
        guard n >= 2 else {
            return Dictionary(uniqueKeysWithValues: layers.map { ($0.imageIndex, GainMap(constant: 1)) })
        }

        // Grid per layer, and the unknown index of each layer's first block.
        struct Grid { var cols: Int; var rows: Int; var offset: Int; var blockW: Double; var blockH: Double }
        var grids: [Grid] = []
        var total = 0
        for layer in layers {
            let block = max(minBlockSize, (max(layer.width, layer.height) + blocksAcross - 1) / blocksAcross)
            let cols = max(1, (layer.width + block - 1) / block)
            let rows = max(1, (layer.height + block - 1) / block)
            grids.append(Grid(cols: cols, rows: rows, offset: total,
                              blockW: Double(layer.width) / Double(cols),
                              blockH: Double(layer.height) / Double(rows)))
            total += cols * rows
        }
        func block(_ g: Grid, col: Int, row: Int) -> Int {
            let c = min(g.cols - 1, Int(Double(col) / g.blockW))
            let r = min(g.rows - 1, Int(Double(row) / g.blockH))
            return g.offset + r * g.cols + c
        }

        var a = [Double](repeating: 0, count: total * total)
        var b = [Double](repeating: 0, count: total)
        let dataScale = 1 / (sigmaN * sigmaN)
        let priorScale = 1 / (priorSigma * priorSigma)

        // Data and prior terms from every overlapping block pair.
        for i in 0..<n {
            for j in (i + 1)..<n {
                let li = layers[i], lj = layers[j]
                let gi = grids[i], gj = grids[j]
                let x0 = max(li.x0, lj.x0), x1 = min(li.x0 + li.width, lj.x0 + lj.width)
                let y0 = max(li.y0, lj.y0), y1 = min(li.y0 + li.height, lj.y0 + lj.height)
                guard x1 > x0, y1 > y0 else { continue }

                // Per (block of i, block of j): count and intensity sums.
                let bi = gi.cols * gi.rows, bj = gj.cols * gj.rows
                var count = [Double](repeating: 0, count: bi * bj)
                var sumI = [Double](repeating: 0, count: bi * bj)
                var sumJ = [Double](repeating: 0, count: bi * bj)
                for y in y0..<y1 {
                    for x in x0..<x1 {
                        let pi = (y - li.y0) * li.width + (x - li.x0)
                        let pj = (y - lj.y0) * lj.width + (x - lj.x0)
                        guard li.validity.pixels[pi] > 0.5, lj.validity.pixels[pj] > 0.5 else { continue }
                        let ki = block(gi, col: x - li.x0, row: y - li.y0) - gi.offset
                        let kj = block(gj, col: x - lj.x0, row: y - lj.y0) - gj.offset
                        let slot = ki * bj + kj
                        count[slot] += 1
                        sumI[slot] += intensity(li, pi)
                        sumJ[slot] += intensity(lj, pj)
                    }
                }
                for ki in 0..<bi {
                    for kj in 0..<bj {
                        let slot = ki * bj + kj
                        let nij = count[slot]
                        // Same floor as the single-gain solve: a few pixels
                        // give a meaningless mean.
                        guard nij > 25 else { continue }
                        let iij = sumI[slot] / nij, iji = sumJ[slot] / nij
                        let p = gi.offset + ki, q = gj.offset + kj
                        a[p * total + p] += nij * (iij * iij * dataScale + priorScale)
                        a[q * total + q] += nij * (iji * iji * dataScale + priorScale)
                        a[p * total + q] -= nij * iij * iji * dataScale
                        a[q * total + p] -= nij * iij * iji * dataScale
                        b[p] += nij * priorScale
                        b[q] += nij * priorScale
                    }
                }
            }
        }

        // Smoothness between 4-neighbor blocks of one image, weighted like
        // the prior over one block's worth of pixels.
        for g in grids {
            let lambda = smoothness * g.blockW * g.blockH * priorScale
            for r in 0..<g.rows {
                for c in 0..<g.cols {
                    let p = g.offset + r * g.cols + c
                    for q in [c + 1 < g.cols ? p + 1 : -1, r + 1 < g.rows ? p + g.cols : -1] where q >= 0 {
                        a[p * total + p] += lambda
                        a[q * total + q] += lambda
                        a[p * total + q] -= lambda
                        a[q * total + p] -= lambda
                    }
                }
            }
        }

        // A weightless prior on every block: keeps the system positive
        // definite for an image with no overlap at all (its gains come out
        // 1) without measurably biasing the rest.
        for p in 0..<total {
            a[p * total + p] += 1
            b[p] += 1
        }

        guard let g = BundleAdjuster.choleskySolve(a, b, n: total) else {
            return Dictionary(uniqueKeysWithValues: layers.map { ($0.imageIndex, GainMap(constant: 1)) })
        }
        var result: [Int: GainMap] = [:]
        for (k, layer) in layers.enumerated() {
            let grid = grids[k]
            let values = g[grid.offset..<(grid.offset + grid.cols * grid.rows)].map { min(max($0, 0.5), 2.0) }
            result[layer.imageIndex] = GainMap(cols: grid.cols, rows: grid.rows, values: Array(values))
        }
        return result
    }

    /// Multiplies the layer's RGB by `gain` in place. Validity and tent are
    /// untouched; clipping to [0, 1] happens once, in the blender's finalize.
    public static func apply(gain: Double, to layer: inout ImageLayer) {
        let g = Float(gain)
        for i in layer.rgb.r.pixels.indices {
            layer.rgb.r.pixels[i] *= g
            layer.rgb.g.pixels[i] *= g
            layer.rgb.b.pixels[i] *= g
        }
    }

    /// Multiplies the layer's RGB by the map's bilinear gain at each pixel,
    /// with the map stretched over the layer's bounding box.
    public static func apply(map: GainMap, to layer: inout ImageLayer) {
        if map.cols == 1 && map.rows == 1 {
            apply(gain: map.values[0], to: &layer)
            return
        }
        let w = layer.width, h = layer.height
        // Column interpolation is the same for every row: precompute it.
        var c0 = [Int](repeating: 0, count: w), c1 = [Int](repeating: 0, count: w)
        var wx = [Double](repeating: 0, count: w)
        for x in 0..<w {
            (c0[x], c1[x], wx[x]) = GainMap.span((Double(x) + 0.5) / Double(w), count: map.cols)
        }
        var rowGain = [Float](repeating: 1, count: w)
        for y in 0..<h {
            let (r0, r1, wy) = GainMap.span((Double(y) + 0.5) / Double(h), count: map.rows)
            for x in 0..<w {
                let top = map.values[r0 * map.cols + c0[x]] * (1 - wx[x]) + map.values[r0 * map.cols + c1[x]] * wx[x]
                let bottom = map.values[r1 * map.cols + c0[x]] * (1 - wx[x]) + map.values[r1 * map.cols + c1[x]] * wx[x]
                rowGain[x] = Float(top * (1 - wy) + bottom * wy)
            }
            let base = y * w
            for x in 0..<w {
                layer.rgb.r.pixels[base + x] *= rowGain[x]
                layer.rgb.g.pixels[base + x] *= rowGain[x]
                layer.rgb.b.pixels[base + x] *= rowGain[x]
            }
        }
    }

    /// Mean of the three channels, the intensity both solvers match.
    @inline(__always)
    static func intensity(_ layer: ImageLayer, _ i: Int) -> Double {
        Double(layer.rgb.r.pixels[i] + layer.rgb.g.pixels[i] + layer.rgb.b.pixels[i]) / 3
    }
}
