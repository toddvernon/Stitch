// Compositing stage 8. Runs on the low-resolution layers before seam
// finding, so the seam cost sees exposure-matched images; the solved gains
// are then applied again to the full-resolution layers before blending.

import Foundation

/// Closed-form gain compensation (Brown & Lowe IJCV 2007 §6): one gain per
/// image minimizing normalized intensity error over all pairwise overlaps,
/// with a prior keeping gains near 1. σ_N = 10/255, σ_g = 0.1 as in the paper.
///
/// Still the paper's single gain per image. DESIGN.md's block-based variant
/// (a grid of gains, bilinearly interpolated, for vignetting and sky
/// gradients) is an open upgrade; multi-band blending hides most of what a
/// single gain leaves behind.
public enum GainCompensator {

    /// Gain per image index. Images with no usable overlap keep 1, and every
    /// gain is clamped to [0.5, 2] so a bad overlap statistic (sky-only
    /// overlap, a moving object) can neither blow out nor black out a photo.
    public static func solve(layers: [ImageLayer]) -> [Int: Double] {
        let n = layers.count
        guard n >= 2 else { return Dictionary(uniqueKeysWithValues: layers.map { ($0.imageIndex, 1.0) }) }

        let sigmaN = 10.0 / 255.0
        let sigmaG = 0.1

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
                        sumA += Double(la.rgb.r.pixels[ia] + la.rgb.g.pixels[ia] + la.rgb.b.pixels[ia]) / 3
                        sumB += Double(lb.rgb.r.pixels[ib] + lb.rgb.g.pixels[ib] + lb.rgb.b.pixels[ib]) / 3
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
}
