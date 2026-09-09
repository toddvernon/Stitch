import Accelerate
import Foundation

// Stage 2 of the pipeline (DESIGN.md): putative descriptor matching between
// two images. PanoramaRecognizer runs it on the SIFT features of every
// candidate pair; the matches feed RANSAC in PairEstimator, which decides
// which of them are geometrically real.

/// A putative correspondence between feature `indexA` in one image and
/// `indexB` in another, with L2 descriptor distance.
public struct FeatureMatch {
    /// Index into the first image's feature array.
    public var indexA: Int
    /// Index into the second image's feature array.
    public var indexB: Int
    /// L2 distance between the two (unit-length) descriptors, 0 to 2.
    public var distance: Float
}

/// Exact 2-nearest-neighbor descriptor matching with Lowe's ratio test.
///
/// Implementation note: descriptors are L2-normalized, so squared distance is
/// 2 - 2·dot, and the whole search is one dense matrix multiply (vDSP). For
/// panorama-sized feature counts this is faster than a k-d tree and exact;
/// the paper's approximate k-d tree only pays off at much larger scales.
public enum DescriptorMatcher {

    /// Matches every feature of `a` against all of `b`, keeping those that
    /// pass the ratio test: nearest distance < `ratio` × second-nearest
    /// (Lowe 2004 §7.1; 0.8 rejects about 90% of false matches while losing
    /// under 5% of correct ones). One-directional (a → b), and several `a`
    /// features may land on the same `b` feature; RANSAC sorts that out.
    public static func match(_ a: [Feature], _ b: [Feature], ratio: Float = 0.8) -> [FeatureMatch] {
        // The ratio needs two candidates on the b side.
        guard a.count >= 1, b.count >= 2 else { return [] }
        let dim = 128
        let nA = a.count, nB = b.count

        var descA = [Float](repeating: 0, count: nA * dim)
        for (i, f) in a.enumerated() {
            descA.replaceSubrange(i * dim..<(i + 1) * dim, with: f.descriptor)
        }
        var descBT = [Float](repeating: 0, count: dim * nB)  // transposed: dim x nB
        for (j, f) in b.enumerated() {
            for d in 0..<dim {
                descBT[d * nB + j] = f.descriptor[d]
            }
        }

        // dots = descA (nA x dim) * descBT (dim x nB)
        var dots = [Float](repeating: 0, count: nA * nB)
        vDSP_mmul(descA, 1, descBT, 1, &dots, 1,
                  vDSP_Length(nA), vDSP_Length(nB), vDSP_Length(dim))

        var matches: [FeatureMatch] = []
        let ratioSq = ratio * ratio
        dots.withUnsafeBufferPointer { buf in
            for i in 0..<nA {
                let row = i * nB
                var best: Float = -2, second: Float = -2
                var bestJ = -1
                for j in 0..<nB {
                    let s = buf[row + j]
                    if s > best {
                        second = best
                        best = s
                        bestJ = j
                    } else if s > second {
                        second = s
                    }
                }
                // squared L2 distances from dot products; if the second-best
                // is also a near-perfect match the feature is ambiguous.
                let d1 = max(2 - 2 * best, 0)
                let d2 = max(2 - 2 * second, 0)
                // d2 ≈ 0 means two identical descriptors in b (duplicate
                // keypoints); nothing can pass a ratio against zero.
                if d2 > 1e-7, d1 < ratioSq * d2 {
                    matches.append(FeatureMatch(indexA: i, indexB: bestJ, distance: sqrtf(d1)))
                }
            }
        }
        return matches
    }
}
