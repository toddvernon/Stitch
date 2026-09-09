import Foundation
import simd

// Step 3 of strip mode (DESIGN.md, "stitch strip"): the strip-mode
// counterpart of PanoramaAligner plus BundleAdjuster. Takes a group that
// PanoramaRecognizer built with the similarity model and places every image
// on the facade plane in one linear solve; StripGeometry renders from it.

/// Placement of every image of a strip on the dominant plane: a similarity
/// from each image's registration-scale pixels into a shared strip frame.
public struct StripAlignment {
    /// Image index → similarity from that image's pixels (top-left origin)
    /// into the strip frame.
    public var transforms: [Int: Similarity]
    /// Robust RMS of inlier residuals after the global solve, px at
    /// registration scale.
    public var finalRMS: Double
}

/// Global alignment for multi-viewpoint strips (DESIGN.md, "stitch strip").
/// Every image gets one similarity onto the facade plane; the RANSAC inliers
/// of every verified pair are solved together as one linear least-squares
/// problem (a similarity is linear in its four parameters), robustified with
/// a few Huber IRLS passes. The gauge is then fixed so the mean rotation is
/// zero and the mean scale is one: "hold the camera level" rather than
/// "walk in a straight line", since photographers are better at the former.
public enum StripAligner {

    /// Huber knee, px at registration scale: a little above feature
    /// localization error plus the depth parallax within the facade band.
    /// Residuals beyond it are weighted linearly instead of squared.
    public static let huberSigma = 3.0
    /// Reweighting passes after the initial L2 solve. Few are needed because
    /// RANSAC already removed the gross outliers.
    public static let irlsPasses = 4

    /// Places every image of `group`. Returns nil for fewer than two images,
    /// no inliers at all, or singular normal equations (which recognition
    /// prevents: every image in a group has at least one verified pair).
    public static func align(group: PanoramaGroup, features: [[Feature]]) -> StripAlignment? {
        let indices = group.imageIndices.sorted()
        guard indices.count >= 2 else { return nil }
        // Compact 0..<k slots for the group's images.
        let slot = Dictionary(uniqueKeysWithValues: indices.enumerated().map { ($1, $0) })

        // Correspondences: (slot i, point in i, slot j, point in j).
        struct Obs {
            var i: Int, p: SIMD2<Double>
            var j: Int, q: SIMD2<Double>
        }
        var obs: [Obs] = []
        for pair in group.pairs {
            guard let si = slot[pair.indexA], let sj = slot[pair.indexB] else { continue }
            let fa = features[pair.indexA], fb = features[pair.indexB]
            for k in pair.geometry.inlierIndices {
                let m = pair.matches[k]
                obs.append(Obs(i: si, p: SIMD2(Double(fa[m.indexA].x), Double(fa[m.indexA].y)),
                               j: sj, q: SIMD2(Double(fb[m.indexB].x), Double(fb[m.indexB].y))))
            }
        }
        guard !obs.isEmpty else { return nil }

        // Gauge for the solve: the best-connected image is the identity and
        // the others are free, unknowns x = (a, b, tx, ty) per free image.
        // This is temporary; once the shape is known the mean-rotation and
        // mean-scale gauge at the end replaces it.
        var degree = [Int](repeating: 0, count: indices.count)
        for o in obs {
            degree[o.i] += 1
            degree[o.j] += 1
        }
        let anchor = degree.indices.max { degree[$0] < degree[$1] }!
        var column = [Int](repeating: -1, count: indices.count)
        var nFree = 0
        for s in indices.indices where s != anchor {
            column[s] = nFree * 4
            nFree += 1
        }
        let nParams = nFree * 4

        var transforms = [Similarity](repeating: .identity, count: indices.count)
        var weights = [Double](repeating: 1, count: obs.count)
        var rms = 0.0

        // Pass 0 is plain least squares; each later pass reuses the previous
        // residuals as Huber weights (IRLS).
        for pass in 0...irlsPasses {
            // Residual r = T_i(p) − T_j(q); each is 2 rows, linear in x.
            // Row for coordinate c of T_i(p): coefficients on (a_i, b_i, tx_i, ty_i)
            // are (p.x, −p.y, 1, 0) for x and (p.y, p.x, 0, 1) for y.
            var ata = [Double](repeating: 0, count: nParams * nParams)
            var atb = [Double](repeating: 0, count: nParams)
            for (k, o) in obs.enumerated() {
                let w = weights[k]
                for c in 0..<2 {
                    var cols: [Int] = []
                    var coef: [Double] = []
                    var rhs = 0.0
                    func add(slot s: Int, point: SIMD2<Double>, sign: Double) {
                        let rowCoef = c == 0 ? [point.x, -point.y, 1.0, 0.0] : [point.y, point.x, 0.0, 1.0]
                        if column[s] >= 0 {
                            for t in 0..<4 {
                                cols.append(column[s] + t)
                                coef.append(sign * rowCoef[t])
                            }
                        } else {
                            // Anchor is the identity: its term moves to the RHS.
                            rhs -= sign * (c == 0 ? point.x : point.y)
                        }
                    }
                    add(slot: o.i, point: o.p, sign: 1)
                    add(slot: o.j, point: o.q, sign: -1)
                    for u in cols.indices {
                        atb[cols[u]] += w * coef[u] * rhs
                        for v in cols.indices {
                            ata[cols[u] * nParams + cols[v]] += w * coef[u] * coef[v]
                        }
                    }
                }
            }
            var x: [Double] = []
            if nParams > 0 {
                guard let solved = BundleAdjuster.choleskySolve(ata, atb, n: nParams) else { return nil }
                x = solved
            }
            for s in indices.indices where column[s] >= 0 {
                let c = column[s]
                transforms[s] = Similarity(a: x[c], b: x[c + 1], tx: x[c + 2], ty: x[c + 3])
            }

            // Huber reweighting for the next pass; robust RMS for reporting.
            var sumSq = 0.0
            var count = 0.0
            for (k, o) in obs.enumerated() {
                let r = length(transforms[o.i].apply(o.p) - transforms[o.j].apply(o.q))
                weights[k] = r <= huberSigma ? 1 : huberSigma / r
                sumSq += weights[k] * r * r
                count += weights[k]
            }
            rms = count > 0 ? sqrt(sumSq / count) : 0
            if pass == irlsPasses { break }
        }

        // Fix the gauge: rotate and scale the whole strip so the mean
        // per-image rotation is zero and the mean scale is one.
        var meanAngle = 0.0, meanLogScale = 0.0
        for t in transforms {
            meanAngle += t.rotation
            meanLogScale += log(max(t.scale, 1e-9))
        }
        meanAngle /= Double(transforms.count)
        meanLogScale /= Double(transforms.count)
        let gauge = Similarity(scale: exp(-meanLogScale), rotation: -meanAngle, translation: SIMD2(0, 0))
        var result: [Int: Similarity] = [:]
        for (s, idx) in indices.enumerated() {
            result[idx] = gauge.composed(with: transforms[s])
        }
        return StripAlignment(transforms: result, finalRMS: rms)
    }
}
