// Scale-space construction for SIFT (Lowe 2004 §3): the Gaussian and DoG
// pyramids `SIFTDetector` searches. Built once per image at registration
// resolution; internal to the Features module.

import Foundation

/// Gaussian scale-space pyramid and its difference-of-Gaussian counterpart,
/// per Lowe (IJCV 2004). Each octave holds scalesPerOctave + 3 Gaussian levels
/// so that DoG extrema can be found across scalesPerOctave layers: S + 2 DoG
/// layers, of which the middle S have neighbors above and below.
struct ScaleSpacePyramid {
    /// gaussians[octave][level], level sigma = initialSigma * 2^(level / S) within the octave.
    let gaussians: [[ImageF]]
    /// dogs[octave][level] = gaussians[octave][level+1] - gaussians[octave][level]
    let dogs: [[ImageF]]
    let config: SIFTConfig

    init(baseImage: ImageF, config: SIFTConfig) {
        self.config = config
        let S = config.scalesPerOctave
        let levelsPerOctave = S + 3

        let minDim = min(baseImage.width, baseImage.height)
        // Smallest octave keeps at least ~16 px on the short side; below that
        // the 5 px border eats the layer and the descriptor window overflows.
        let nOctaves = max(1, Int(floor(log2(Double(minDim)))) - 3)

        // Per-level incremental blur so level i has total sigma initialSigma * k^i.
        // Gaussians compose in quadrature, so each step blurs the previous
        // level by sqrt(σ_i² − σ_{i−1}²) rather than re-blurring the base;
        // this keeps the kernels small.
        let k = powf(2, 1 / Float(S))
        var sigmaIncrements = [Float](repeating: 0, count: levelsPerOctave)
        var prevTotal = config.initialSigma
        for i in 1..<levelsPerOctave {
            let total = config.initialSigma * powf(k, Float(i))
            sigmaIncrements[i] = sqrtf(total * total - prevTotal * prevTotal)
            prevTotal = total
        }

        var gaussians: [[ImageF]] = []
        var dogs: [[ImageF]] = []
        var octaveBase = baseImage
        for _ in 0..<nOctaves {
            var levels: [ImageF] = [octaveBase]
            for i in 1..<levelsPerOctave {
                levels.append(Convolution.gaussianBlur(levels[i - 1], sigma: sigmaIncrements[i]))
            }
            var octaveDogs: [ImageF] = []
            for i in 0..<(levelsPerOctave - 1) {
                octaveDogs.append(Convolution.subtract(levels[i + 1], levels[i]))
            }
            gaussians.append(levels)
            dogs.append(octaveDogs)
            // Level S has exactly twice the base sigma; decimating it gives the
            // next octave's base at initialSigma.
            octaveBase = Convolution.downsample2(levels[S])
        }
        self.gaussians = gaussians
        self.dogs = dogs
    }
}
