import Foundation
import simd

/// Which motion model relates a pair of images.
public enum PairModel: String, Sendable {
    /// Rotation about the lens: full 8-DOF homography (panoramas).
    case homography
    /// Translation along a facade: 4-DOF similarity on the dominant plane
    /// (multi-viewpoint strips). Two-point samples make RANSAC robust when
    /// only a handful of the putative matches are real.
    case similarity
}

/// Geometry of a verified (or rejected) image pair.
public struct PairGeometry {
    /// Maps image-A pixel coordinates to image-B pixel coordinates. For the
    /// similarity model the last row is (0, 0, 1); see `similarity`.
    public var homography: Homography
    /// Indices into the input match array that are RANSAC inliers.
    public var inlierIndices: [Int]
    /// Matches whose A-side feature projects inside image B (n_f in the paper).
    public var overlapMatchCount: Int
    /// Brown & Lowe probabilistic verification: n_i > α + β·n_f (α=8.0, β=0.3)
    /// for homographies; the strip rule for similarities (see `PairEstimator`).
    public var isVerified: Bool

    public var similarity: Similarity? { Similarity(homography: homography) }
}

/// RANSAC estimation over putative matches, followed by pair verification
/// (Brown & Lowe IJCV 2007 §3 for the rotational model).
public enum PairEstimator {

    public static let verificationAlpha = 8.0
    public static let verificationBeta = 0.3

    /// Strip verification: a similarity found from two-point samples with a
    /// depth-tolerant threshold has essentially no chance of collecting this
    /// many spurious inliers, so a flat minimum is enough — plus sanity limits
    /// on scale and rotation, since strips are shot square to the facade.
    public static let similarityMinInliers = 8
    public static let similarityScaleRange = 0.5...2.0
    public static let similarityMaxRotation = 20.0 * .pi / 180

    public static func estimate(featuresA: [Feature],
                                featuresB: [Feature],
                                matches: [FeatureMatch],
                                imageBWidth: Int,
                                imageBHeight: Int,
                                model: PairModel = .homography,
                                iterations: Int = 500,
                                inlierThreshold: Double? = nil,
                                seed: UInt64 = 0x5EED) -> PairGeometry? {
        let sampleSize = model == .homography ? 4 : 2
        guard matches.count >= sampleSize else { return nil }

        let ptsA = matches.map { SIMD2<Double>(Double(featuresA[$0.indexA].x), Double(featuresA[$0.indexA].y)) }
        let ptsB = matches.map { SIMD2<Double>(Double(featuresB[$0.indexB].x), Double(featuresB[$0.indexB].y)) }
        // The similarity threshold is loose on purpose: a facade "plane" is a
        // band of houses and trees at slightly different depths, whose motion
        // parallax must not split the true matches into competing models.
        let threshold = inlierThreshold ?? (model == .homography
            ? 3.0 : max(3.0, 0.006 * Double(max(imageBWidth, imageBHeight))))
        let threshSq = threshold * threshold

        func fit(_ idx: [Int]) -> Homography? {
            switch model {
            case .homography:
                return HomographyEstimator.dlt(from: idx.map { ptsA[$0] }, to: idx.map { ptsB[$0] })
            case .similarity:
                guard let s = Similarity.fit(from: idx.map { ptsA[$0] }, to: idx.map { ptsB[$0] }),
                      similarityScaleRange.contains(s.scale),
                      abs(s.rotation) < similarityMaxRotation else { return nil }
                return s.homography
            }
        }

        var rng = SplitMix64(seed: seed)
        var bestInliers: [Int] = []

        for _ in 0..<iterations {
            let sample = randomSample(sampleSize, count: matches.count, rng: &rng)
            guard let h = fit(sample) else { continue }
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
        guard bestInliers.count >= sampleSize else { return nil }

        // Refit on all inliers of the best model, then recollect inliers once.
        guard let refined = fit(bestInliers) else { return nil }
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

        let verified: Bool
        switch model {
        case .homography:
            verified = Double(inliers.count) > verificationAlpha + verificationBeta * Double(overlapCount)
        case .similarity:
            verified = inliers.count >= similarityMinInliers
        }
        return PairGeometry(homography: refined,
                            inlierIndices: inliers,
                            overlapMatchCount: overlapCount,
                            isVerified: verified)
    }

    private static func randomSample(_ size: Int, count: Int, rng: inout SplitMix64) -> [Int] {
        var picked: [Int] = []
        picked.reserveCapacity(size)
        while picked.count < size {
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
