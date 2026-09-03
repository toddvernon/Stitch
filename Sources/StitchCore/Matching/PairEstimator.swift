import Foundation
import simd

/// Geometry of a verified (or rejected) image pair.
public struct PairGeometry {
    /// Maps image-A pixel coordinates to image-B pixel coordinates.
    public var homography: Homography
    /// Indices into the input match array that are RANSAC inliers.
    public var inlierIndices: [Int]
    /// Matches whose A-side feature projects inside image B (n_f in the paper).
    public var overlapMatchCount: Int
    /// Brown & Lowe probabilistic verification: n_i > α + β·n_f (α=8.0, β=0.3).
    public var isVerified: Bool
}

/// RANSAC homography estimation over putative matches, followed by the
/// Brown & Lowe probabilistic image-match verification (IJCV 2007 §3).
public enum PairEstimator {

    public static let verificationAlpha = 8.0
    public static let verificationBeta = 0.3

    public static func estimate(featuresA: [Feature],
                                featuresB: [Feature],
                                matches: [FeatureMatch],
                                imageBWidth: Int,
                                imageBHeight: Int,
                                iterations: Int = 500,
                                inlierThreshold: Double = 3.0,
                                seed: UInt64 = 0x5EED) -> PairGeometry? {
        guard matches.count >= 4 else { return nil }

        let ptsA = matches.map { SIMD2<Double>(Double(featuresA[$0.indexA].x), Double(featuresA[$0.indexA].y)) }
        let ptsB = matches.map { SIMD2<Double>(Double(featuresB[$0.indexB].x), Double(featuresB[$0.indexB].y)) }
        let threshSq = inlierThreshold * inlierThreshold

        var rng = SplitMix64(seed: seed)
        var bestInliers: [Int] = []

        for _ in 0..<iterations {
            let sample = randomSample4(count: matches.count, rng: &rng)
            guard let h = HomographyEstimator.dlt(from: sample.map { ptsA[$0] },
                                                 to: sample.map { ptsB[$0] }) else { continue }
            var inliers: [Int] = []
            for k in 0..<matches.count {
                let p = HomographyEstimator.project(h, ptsA[k])
                if length_squared(p - ptsB[k]) < threshSq {
                    inliers.append(k)
                }
            }
            if inliers.count > bestInliers.count {
                bestInliers = inliers
            }
        }
        guard bestInliers.count >= 4 else { return nil }

        // Refit on all inliers of the best model, then recollect inliers once.
        guard let refined = HomographyEstimator.dlt(from: bestInliers.map { ptsA[$0] },
                                                    to: bestInliers.map { ptsB[$0] }) else { return nil }
        var inliers: [Int] = []
        var overlapCount = 0
        let w = Double(imageBWidth), h = Double(imageBHeight)
        for k in 0..<matches.count {
            let p = HomographyEstimator.project(refined, ptsA[k])
            if p.x >= 0, p.x < w, p.y >= 0, p.y < h {
                overlapCount += 1
            }
            if length_squared(p - ptsB[k]) < threshSq {
                inliers.append(k)
            }
        }

        let verified = Double(inliers.count) > verificationAlpha + verificationBeta * Double(overlapCount)
        return PairGeometry(homography: refined,
                            inlierIndices: inliers,
                            overlapMatchCount: overlapCount,
                            isVerified: verified)
    }

    private static func randomSample4(count: Int, rng: inout SplitMix64) -> [Int] {
        var picked: [Int] = []
        picked.reserveCapacity(4)
        while picked.count < 4 {
            let i = Int(rng.next() % UInt64(count))
            if !picked.contains(i) { picked.append(i) }
        }
        return picked
    }
}

/// Small deterministic RNG so RANSAC results are reproducible in tests.
public struct SplitMix64: RandomNumberGenerator {
    private var state: UInt64
    public init(seed: UInt64) { state = seed }
    public mutating func next() -> UInt64 {
        state &+= 0x9E3779B97F4A7C15
        var z = state
        z = (z ^ (z >> 30)) &* 0xBF58476D1CE4E5B9
        z = (z ^ (z >> 27)) &* 0x94D049BB133111EB
        return z ^ (z >> 31)
    }
}
