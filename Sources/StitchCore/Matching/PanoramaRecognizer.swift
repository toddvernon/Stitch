import Foundation

// Stages 2 to 4 of the pipeline (DESIGN.md): match every image pair, verify
// the promising ones with RANSAC, and group images into panoramas by
// connected components. The groups go to PanoramaAligner (rotational) or
// StripAligner (strip mode); images that verify with nothing are dropped.

/// A verified geometric relationship between two images in a set.
public struct VerifiedPair {
    /// Image indices into the original set, with `indexA < indexB`.
    public var indexA: Int
    public var indexB: Int
    /// All putative matches of the pair; `geometry.inlierIndices` picks out
    /// the ones the model explains.
    public var matches: [FeatureMatch]
    /// The fitted pair model (A pixels → B pixels) and its verification.
    public var geometry: PairGeometry
}

/// One recognized panorama: a connected component of verified image pairs.
public struct PanoramaGroup {
    /// Every image in the component, in original index order (which says
    /// nothing about shooting order; the aligners work that out).
    public var imageIndices: [Int]
    /// Every verified pair whose two images are both in the component.
    public var pairs: [VerifiedPair]
}

/// Brown & Lowe panorama recognition (IJCV 2007 §3): match candidate pairs,
/// verify them geometrically, and take connected components. Images in no
/// component (noise images) are simply absent from the result. The same
/// procedure recognizes multi-viewpoint strips when run with the similarity
/// model (see `PairModel`).
public enum PanoramaRecognizer {

    /// Candidate pairs per image, by raw match count (m in the paper, §3.1).
    public static let candidatesPerImage = 6

    /// Recognizes every panorama (or strip, with `model == .similarity`)
    /// among `features`, one array per image; `imageSizes` are at the same
    /// registration scale the features were detected at. Groups come back
    /// largest first; images belonging to no group are simply absent.
    public static func recognize(features: [[Feature]],
                                 imageSizes: [(width: Int, height: Int)],
                                 model: PairModel = .homography) -> [PanoramaGroup] {
        let n = features.count
        guard n >= 2 else { return [] }

        // Putative matches for every unordered pair. For panorama-scale n the
        // quadratic pair loop is cheap next to detection; candidate selection
        // below limits the RANSAC work like the paper's m-best rule.
        var rawMatches = [[FeatureMatch]?](repeating: nil, count: n * n)
        var matchCounts = [[(j: Int, count: Int)]](repeating: [], count: n)
        for i in 0..<n {
            for j in (i + 1)..<n {
                let m = DescriptorMatcher.match(features[i], features[j])
                rawMatches[i * n + j] = m
                matchCounts[i].append((j, m.count))
                matchCounts[j].append((i, m.count))
            }
        }

        // Keep each image's top-m candidates. A pair survives if either image
        // lists it, so a hub image with many neighbors doesn't starve them.
        var candidate = Set<Int>()  // encoded i*n+j with i<j
        for i in 0..<n {
            // Under 4 raw matches nothing can verify under either model.
            for (j, count) in matchCounts[i].sorted(by: { $0.count > $1.count }).prefix(candidatesPerImage)
            where count >= 4 {
                candidate.insert(min(i, j) * n + max(i, j))
            }
        }

        // RANSAC + verification on candidates.
        var pairs: [VerifiedPair] = []
        for code in candidate.sorted() {
            let i = code / n, j = code % n
            guard let matches = rawMatches[code], matches.count >= 4 else { continue }
            guard let geometry = PairEstimator.estimate(featuresA: features[i], featuresB: features[j],
                                                        matches: matches,
                                                        imageBWidth: imageSizes[j].width,
                                                        imageBHeight: imageSizes[j].height,
                                                        model: model),
                  geometry.isVerified else { continue }
            pairs.append(VerifiedPair(indexA: i, indexB: j, matches: matches, geometry: geometry))
        }

        // Connected components via union-find (paper §3.3), with path
        // compression so repeated finds stay cheap.
        var parent = Array(0..<n)
        func find(_ x: Int) -> Int {
            var root = x
            while parent[root] != root { root = parent[root] }
            var cur = x
            while parent[cur] != root {
                let next = parent[cur]
                parent[cur] = root
                cur = next
            }
            return root
        }
        for p in pairs {
            parent[find(p.indexA)] = find(p.indexB)
        }

        // Collect pairs and then images per root. Only roots that own a pair
        // get an entry, so single-image components (noise images) never
        // appear; the size filter below just states the contract.
        var groups: [Int: PanoramaGroup] = [:]
        for p in pairs {
            let root = find(p.indexA)
            groups[root, default: PanoramaGroup(imageIndices: [], pairs: [])].pairs.append(p)
        }
        for i in 0..<n {
            let root = find(i)
            if groups[root] != nil {
                groups[root]!.imageIndices.append(i)
            }
        }
        return groups.values
            .sorted { $0.imageIndices.count > $1.imageIndices.count }
            .filter { $0.imageIndices.count >= 2 }
    }
}
