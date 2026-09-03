import Accelerate
import Foundation

/// A putative correspondence between feature `indexA` in one image and
/// `indexB` in another, with L2 descriptor distance.
public struct FeatureMatch {
    public var indexA: Int
    public var indexB: Int
    public var distance: Float
}

/// Exact 2-nearest-neighbor descriptor matching with Lowe's ratio test.
///
/// Implementation note: descriptors are L2-normalized, so squared distance is
/// 2 - 2·dot, and the whole search is one dense matrix multiply (vDSP). For
/// panorama-sized feature counts this is faster than a k-d tree and exact;
/// the paper's approximate k-d tree only pays off at much larger scales.
public enum DescriptorMatcher {

    public static func match(_ a: [Feature], _ b: [Feature], ratio: Float = 0.8) -> [FeatureMatch] {
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
                if d2 > 1e-7, d1 < ratioSq * d2 {
                    matches.append(FeatureMatch(indexA: i, indexB: bestJ, distance: sqrtf(d1)))
                }
            }
        }
        return matches
    }
}
