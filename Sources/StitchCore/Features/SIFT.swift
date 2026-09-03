import Foundation

public struct SIFTConfig {
    /// S in Lowe's paper: DoG layers searched for extrema per octave.
    public var scalesPerOctave = 3
    /// Blur of the base pyramid level.
    public var initialSigma: Float = 1.6
    /// Blur assumed already present in the input image.
    public var assumedBlur: Float = 0.5
    /// Double the input before building the pyramid (more features, 4x cost).
    public var doubleImage = false
    /// Contrast threshold, OpenCV convention: reject if |D(x̂)| * S < this.
    public var contrastThreshold: Float = 0.04
    /// Edge response ratio r; reject if tr²/det ≥ (r+1)²/r.
    public var edgeThreshold: Float = 10
    /// Keep only the strongest N features (by |DoG| response), nil = keep all.
    public var maxFeatures: Int? = nil

    public init() {}
}

/// A detected feature: position/scale/orientation in input-image coordinates,
/// plus a 128-dimensional SIFT descriptor (L2-normalized, 0.2-clipped).
public struct Feature {
    public var x: Float
    public var y: Float
    public var scale: Float
    public var orientation: Float
    public var response: Float
    public var descriptor: [Float]
}

public protocol FeatureExtractor {
    func detect(in image: ImageF) -> [Feature]
}

public struct SIFTDetector: FeatureExtractor {
    public let config: SIFTConfig
    public init(config: SIFTConfig = SIFTConfig()) {
        self.config = config
    }

    private static let imageBorder = 5
    private static let maxInterpSteps = 5
    private static let orientationBins = 36
    private static let orientationSigmaFactor: Float = 1.5
    private static let orientationPeakRatio: Float = 0.8
    private static let descriptorWidth = 4     // d: 4x4 spatial grid
    private static let descriptorOriBins = 8   // n: orientations per cell
    private static let descriptorSclFactor: Float = 3.0
    private static let descriptorMagThreshold: Float = 0.2

    public func detect(in image: ImageF) -> [Feature] {
        let base = makeBaseImage(image)
        let pyramid = ScaleSpacePyramid(baseImage: base, config: config)
        var features = findFeatures(pyramid: pyramid)
        if config.doubleImage {
            for i in features.indices {
                features[i].x *= 0.5
                features[i].y *= 0.5
                features[i].scale *= 0.5
            }
        }
        if let cap = config.maxFeatures, features.count > cap {
            features.sort { abs($0.response) > abs($1.response) }
            features.removeLast(features.count - cap)
        }
        return features
    }

    private func makeBaseImage(_ image: ImageF) -> ImageF {
        var img = image
        var assumed = config.assumedBlur
        if config.doubleImage {
            img = Convolution.upsample2(img)
            assumed *= 2
        }
        let sigmaDiff = sqrtf(max(config.initialSigma * config.initialSigma - assumed * assumed, 0.01))
        return Convolution.gaussianBlur(img, sigma: sigmaDiff)
    }

    // MARK: - Extrema detection

    private struct Candidate {
        var octave: Int
        var layer: Int          // refined integer DoG layer
        var x: Int              // refined integer position (octave coords)
        var y: Int
        var subX: Float         // sub-pixel offsets from interpolation
        var subY: Float
        var subLayer: Float
        var response: Float
        var octaveScale: Float  // sigma within the octave, for orientation/descriptor windows
    }

    private func findFeatures(pyramid: ScaleSpacePyramid) -> [Feature] {
        let S = config.scalesPerOctave
        let border = Self.imageBorder
        let preThreshold = 0.5 * config.contrastThreshold / Float(S)
        var features: [Feature] = []

        for o in 0..<pyramid.dogs.count {
            let octave = pyramid.dogs[o]
            let w = octave[0].width, h = octave[0].height
            guard w > 2 * border, h > 2 * border else { continue }
            for l in 1...S {
                octave[l - 1].pixels.withUnsafeBufferPointer { prev in
                octave[l].pixels.withUnsafeBufferPointer { cur in
                octave[l + 1].pixels.withUnsafeBufferPointer { next in
                    for y in border..<(h - border) {
                        let row = y * w
                        for x in border..<(w - border) {
                            let i = row + x
                            let v = cur[i]
                            guard abs(v) > preThreshold else { continue }
                            guard isExtremum(v: v, i: i, w: w, prev: prev, cur: cur, next: next) else { continue }
                            guard var cand = refine(pyramid: pyramid, octave: o, layer: l, x: x, y: y) else { continue }
                            cand.octave = o
                            appendFeatures(for: cand, pyramid: pyramid, into: &features)
                        }
                    }
                }}}
            }
        }
        return features
    }

    @inline(__always)
    private func isExtremum(v: Float, i: Int, w: Int,
                            prev: UnsafeBufferPointer<Float>,
                            cur: UnsafeBufferPointer<Float>,
                            next: UnsafeBufferPointer<Float>) -> Bool {
        let offsets = [-w - 1, -w, -w + 1, -1, 0, 1, w - 1, w, w + 1]
        if v > 0 {
            for d in offsets {
                if v < prev[i + d] || v < next[i + d] { return false }
                if d != 0 && v < cur[i + d] { return false }
            }
        } else {
            for d in offsets {
                if v > prev[i + d] || v > next[i + d] { return false }
                if d != 0 && v > cur[i + d] { return false }
            }
        }
        return true
    }

    /// Sub-pixel/sub-scale refinement by quadratic fit (Brown & Lowe 2002 as used
    /// in Lowe 2004 §4), plus contrast and edge-response rejection.
    private func refine(pyramid: ScaleSpacePyramid, octave: Int, layer: Int, x: Int, y: Int) -> Candidate? {
        let S = config.scalesPerOctave
        let border = Self.imageBorder
        let dogs = pyramid.dogs[octave]
        let w = dogs[0].width, h = dogs[0].height

        var l = layer, px = x, py = y
        var dx: Float = 0, dy: Float = 0, ds: Float = 0

        for step in 0...Self.maxInterpSteps {
            let prev = dogs[l - 1], cur = dogs[l], next = dogs[l + 1]
            @inline(__always) func c(_ ix: Int, _ iy: Int) -> Float { cur[ix, iy] }

            let gx = (c(px + 1, py) - c(px - 1, py)) * 0.5
            let gy = (c(px, py + 1) - c(px, py - 1)) * 0.5
            let gs = (next[px, py] - prev[px, py]) * 0.5

            let v2 = c(px, py) * 2
            let dxx = c(px + 1, py) + c(px - 1, py) - v2
            let dyy = c(px, py + 1) + c(px, py - 1) - v2
            let dss = next[px, py] + prev[px, py] - v2
            let dxy = (c(px + 1, py + 1) - c(px - 1, py + 1) - c(px + 1, py - 1) + c(px - 1, py - 1)) * 0.25
            let dxs = (next[px + 1, py] - next[px - 1, py] - prev[px + 1, py] + prev[px - 1, py]) * 0.25
            let dys = (next[px, py + 1] - next[px, py - 1] - prev[px, py + 1] + prev[px, py - 1]) * 0.25

            guard let sol = Self.solve3x3(
                a: (dxx, dxy, dxs,
                    dxy, dyy, dys,
                    dxs, dys, dss),
                b: (-gx, -gy, -gs)) else { return nil }
            (dx, dy, ds) = sol

            if abs(dx) < 0.5 && abs(dy) < 0.5 && abs(ds) < 0.5 { break }
            if step == Self.maxInterpSteps { return nil }

            px += Int(dx.rounded())
            py += Int(dy.rounded())
            l += Int(ds.rounded())
            guard l >= 1, l <= S,
                  px >= border, px < w - border,
                  py >= border, py < h - border else { return nil }
        }

        // Contrast check at the interpolated position.
        let cur = dogs[l]
        let gx = (cur[px + 1, py] - cur[px - 1, py]) * 0.5
        let gy = (cur[px, py + 1] - cur[px, py - 1]) * 0.5
        let gs = (dogs[l + 1][px, py] - dogs[l - 1][px, py]) * 0.5
        let contrast = cur[px, py] + 0.5 * (gx * dx + gy * dy + gs * ds)
        guard abs(contrast) * Float(S) >= config.contrastThreshold else { return nil }

        // Edge response: ratio of principal curvatures of the 2D spatial Hessian.
        let v2 = cur[px, py] * 2
        let dxx = cur[px + 1, py] + cur[px - 1, py] - v2
        let dyy = cur[px, py + 1] + cur[px, py - 1] - v2
        let dxy = (cur[px + 1, py + 1] - cur[px - 1, py + 1] - cur[px + 1, py - 1] + cur[px - 1, py - 1]) * 0.25
        let trace = dxx + dyy
        let det = dxx * dyy - dxy * dxy
        let r = config.edgeThreshold
        guard det > 0, trace * trace * r < (r + 1) * (r + 1) * det else { return nil }

        let octaveScale = config.initialSigma * powf(2, (Float(l) + ds) / Float(S))
        return Candidate(octave: octave, layer: l, x: px, y: py,
                         subX: dx, subY: dy, subLayer: ds,
                         response: contrast, octaveScale: octaveScale)
    }

    private static func solve3x3(a: (Float, Float, Float, Float, Float, Float, Float, Float, Float),
                                 b: (Float, Float, Float)) -> (Float, Float, Float)? {
        let (a11, a12, a13, a21, a22, a23, a31, a32, a33) = a
        let det = a11 * (a22 * a33 - a23 * a32)
                - a12 * (a21 * a33 - a23 * a31)
                + a13 * (a21 * a32 - a22 * a31)
        guard abs(det) > 1e-12 else { return nil }
        let (b1, b2, b3) = b
        let x = (b1 * (a22 * a33 - a23 * a32) - a12 * (b2 * a33 - a23 * b3) + a13 * (b2 * a32 - a22 * b3)) / det
        let y = (a11 * (b2 * a33 - a23 * b3) - b1 * (a21 * a33 - a23 * a31) + a13 * (a21 * b3 - b2 * a31)) / det
        let z = (a11 * (a22 * b3 - b2 * a32) - a12 * (a21 * b3 - b2 * a31) + b1 * (a21 * a32 - a22 * a31)) / det
        return (x, y, z)
    }

    // MARK: - Orientation and descriptor

    private func appendFeatures(for cand: Candidate, pyramid: ScaleSpacePyramid, into features: inout [Feature]) {
        let gauss = pyramid.gaussians[cand.octave][cand.layer]
        let angles = dominantOrientations(img: gauss, x: cand.x, y: cand.y, octaveScale: cand.octaveScale)
        guard !angles.isEmpty else { return }

        let octaveFactor = powf(2, Float(cand.octave))
        let fx = (Float(cand.x) + cand.subX) * octaveFactor
        let fy = (Float(cand.y) + cand.subY) * octaveFactor
        let scale = cand.octaveScale * octaveFactor
        let px = Float(cand.x) + cand.subX
        let py = Float(cand.y) + cand.subY

        for angle in angles {
            let desc = descriptor(img: gauss, x: px, y: py, angle: angle, octaveScale: cand.octaveScale)
            features.append(Feature(x: fx, y: fy, scale: scale, orientation: angle,
                                    response: cand.response, descriptor: desc))
        }
    }

    private func dominantOrientations(img: ImageF, x: Int, y: Int, octaveScale: Float) -> [Float] {
        let nBins = Self.orientationBins
        let sigma = Self.orientationSigmaFactor * octaveScale
        let radius = Int((3 * sigma).rounded())
        guard radius >= 1 else { return [] }
        var hist = [Float](repeating: 0, count: nBins)
        let expDenom = 2 * sigma * sigma

        for i in -radius...radius {
            let py = y + i
            guard py > 0, py < img.height - 1 else { continue }
            for j in -radius...radius {
                let px = x + j
                guard px > 0, px < img.width - 1 else { continue }
                let dx = img[px + 1, py] - img[px - 1, py]
                let dy = img[px, py + 1] - img[px, py - 1]
                let mag = sqrtf(dx * dx + dy * dy)
                let weight = expf(-Float(i * i + j * j) / expDenom)
                var theta = atan2f(dy, dx)
                if theta < 0 { theta += 2 * .pi }
                var bin = Int(theta / (2 * .pi) * Float(nBins))
                if bin >= nBins { bin = 0 }
                hist[bin] += weight * mag
            }
        }

        // Two passes of circular [1,4,6,4,1]/16 smoothing.
        for _ in 0..<2 {
            let src = hist
            for i in 0..<nBins {
                hist[i] = (src[(i + nBins - 2) % nBins] + src[(i + 2) % nBins]) * (1.0 / 16)
                        + (src[(i + nBins - 1) % nBins] + src[(i + 1) % nBins]) * (4.0 / 16)
                        + src[i] * (6.0 / 16)
            }
        }

        guard let maxVal = hist.max(), maxVal > 0 else { return [] }
        var angles: [Float] = []
        for i in 0..<nBins {
            let left = hist[(i + nBins - 1) % nBins]
            let right = hist[(i + 1) % nBins]
            if hist[i] > left, hist[i] > right, hist[i] >= Self.orientationPeakRatio * maxVal {
                var bin = Float(i) + 0.5 * (left - right) / (left - 2 * hist[i] + right)
                if bin < 0 { bin += Float(nBins) }
                if bin >= Float(nBins) { bin -= Float(nBins) }
                angles.append(bin / Float(nBins) * 2 * .pi)
            }
        }
        return angles
    }

    private func descriptor(img: ImageF, x: Float, y: Float, angle: Float, octaveScale: Float) -> [Float] {
        let d = Self.descriptorWidth
        let n = Self.descriptorOriBins
        let histWidth = Self.descriptorSclFactor * octaveScale
        var radius = Int((histWidth * sqrtf(2) * Float(d + 1) * 0.5).rounded())
        radius = min(radius, Int(sqrtf(Float(img.width * img.width + img.height * img.height))))

        let cosT = cosf(angle) / histWidth
        let sinT = sinf(angle) / histWidth
        let binsPerRad = Float(n) / (2 * .pi)
        let expScale: Float = -2.0 / Float(d * d)
        let cx = Int(x.rounded()), cy = Int(y.rounded())

        // (d+2)^2 spatial bins with a one-bin margin for interpolation; orientation wraps.
        var hist = [Float](repeating: 0, count: (d + 2) * (d + 2) * n)
        @inline(__always) func idx(_ r: Int, _ c: Int, _ o: Int) -> Int { (r * (d + 2) + c) * n + o }

        for i in -radius...radius {
            let py = cy + i
            guard py > 0, py < img.height - 1 else { continue }
            for j in -radius...radius {
                let px = cx + j
                guard px > 0, px < img.width - 1 else { continue }

                let cRot = Float(j) * cosT - Float(i) * sinT
                let rRot = Float(j) * sinT + Float(i) * cosT
                let rBin = rRot + Float(d) / 2 - 0.5
                let cBin = cRot + Float(d) / 2 - 0.5
                guard rBin > -1, rBin < Float(d), cBin > -1, cBin < Float(d) else { continue }

                let dx = img[px + 1, py] - img[px - 1, py]
                let dy = img[px, py + 1] - img[px, py - 1]
                var theta = atan2f(dy, dx) - angle
                while theta < 0 { theta += 2 * .pi }
                while theta >= 2 * .pi { theta -= 2 * .pi }

                let weight = expf((cRot * cRot + rRot * rRot) * expScale)
                let mag = sqrtf(dx * dx + dy * dy) * weight

                let oBin = theta * binsPerRad
                let r0 = Int(floorf(rBin)), c0 = Int(floorf(cBin))
                var o0 = Int(floorf(oBin))
                let dr = rBin - Float(r0), dc = cBin - Float(c0), dob = oBin - Float(o0)
                if o0 >= n { o0 -= n }

                for (ri, rw) in [(r0, 1 - dr), (r0 + 1, dr)] {
                    for (ci, cw) in [(c0, 1 - dc), (c0 + 1, dc)] {
                        let w0 = mag * rw * cw
                        hist[idx(ri + 1, ci + 1, o0)] += w0 * (1 - dob)
                        hist[idx(ri + 1, ci + 1, (o0 + 1) % n)] += w0 * dob
                    }
                }
            }
        }

        var desc = [Float](repeating: 0, count: d * d * n)
        var k = 0
        for r in 1...d {
            for c in 1...d {
                for o in 0..<n {
                    desc[k] = hist[idx(r, c, o)]
                    k += 1
                }
            }
        }

        normalize(&desc)
        var clipped = false
        for i in desc.indices where desc[i] > Self.descriptorMagThreshold {
            desc[i] = Self.descriptorMagThreshold
            clipped = true
        }
        if clipped { normalize(&desc) }
        return desc
    }

    private func normalize(_ v: inout [Float]) {
        var sum: Float = 0
        for x in v { sum += x * x }
        let norm = sqrtf(sum)
        guard norm > 1e-12 else { return }
        for i in v.indices { v[i] /= norm }
    }
}
