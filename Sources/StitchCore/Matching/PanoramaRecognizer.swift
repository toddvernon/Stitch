import Foundation

/// A verified geometric relationship between two images in a set.
public struct VerifiedPair {
    public var indexA: Int
    public var indexB: Int
    public var matches: [FeatureMatch]
    public var geometry: PairGeometry
}

/// One recognized panorama: a connected component of verified image pairs.
public struct PanoramaGroup {
    public var imageIndices: [Int]
    public var pairs: [VerifiedPair]
}

/// Brown & Lowe panorama recognition (IJCV 2007 §3): match candidate pairs,
/// verify them geometrically, and take connected components. Images in no
/// component (noise images) are simply absent from the result.
public enum PanoramaRecognizer {

    /// Candidate pairs per image, by raw match count (m in the paper).
    public static let candidatesPerImage = 6

    public static func recognize(features: [[Feature]],
                                 imageSizes: [(width: Int, height: Int)]) -> [PanoramaGroup] {
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

        // Keep each image's top-m candidates.
        var candidate = Set<Int>()  // encoded i*n+j with i<j
        for i in 0..<n {
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
                                                        imageBHeight: imageSizes[j].height),
                  geometry.isVerified else { continue }
            pairs.append(VerifiedPair(indexA: i, indexB: j, matches: matches, geometry: geometry))
        }

        // Connected components via union-find.
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
