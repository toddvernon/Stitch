// Compositing stages 8 through 10 (DESIGN.md) as one driver: the only code
// that orders gain, seams, and blending, and the only place full-resolution
// sources are loaded. Stitcher calls compose(cameras:...) for panoramas and
// compose(source:) with a StripGeometry for strips.

import Foundation

/// Milestone-6 compositing pipeline: project layers, compensate gain, find
/// graph-cut seams at working resolution, multi-band blend at full resolution,
/// and crop to the largest fully-covered rectangle. Generic over the output
/// space via `LayerSource`, so panoramas and strips share it.
public enum Compositor {

    public struct Options {
        /// Final output width in pixels; height follows from the geometry.
        public var outputWidth = 4000
        /// Working width for gain/seam estimation. Kept small deliberately:
        /// graph-cut cost grows steeply with node count, and seam placement
        /// error of a few working pixels disappears under multi-band blending.
        public var seamWidth = 700
        /// Long-side cap when loading sources for the low-res gain/seam pass.
        public var seamSourceDimension = 800
        /// Pyramid depth for multi-band blending. 5 is Brown & Lowe's figure
        /// and suits rotational panoramas, whose exposure differences gain
        /// compensation mostly removes; strips use 8 (see DESIGN.md) because
        /// their residual differences are local and need spreading further.
        public var blendLevels = 5
        /// Crop to the largest rectangle with no uncovered pixels.
        public var crop = true
        /// nil = auto: Pannini under 160° of span, spherical above.
        public var projection: PanoProjection? = nil
        /// Seam data term favoring the image whose center is nearest each
        /// pixel (see `SeamFinder`). 0 for panoramas; strips set it so each
        /// region comes from the photo taken most directly in front of it.
        public var seamLocalityWeight = 0.0
        public init() {}
    }

    /// The auto rule: Pannini flatters wide architecture but degrades past
    /// ~150-160°; spherical is correct at any span.
    public static func resolveProjection(_ choice: PanoProjection?,
                                         cameras: [Int: Camera]) -> PanoProjection {
        if let choice { return choice }
        guard let probe = PanoGeometry(cameras: cameras, outputWidth: 1000,
                                       projection: .spherical) else { return .spherical }
        let spanDegrees = (probe.thetaMax - probe.thetaMin) * 180 / .pi
        return spanDegrees < 160 ? .pannini : .spherical
    }

    public struct Result {
        public var image: RGBImage
        /// Solved gain per image index, for logging and diagnostics.
        public var gains: [Int: Double]
    }

    /// Rotational panorama: builds the `PanoLayerSource` and composites.
    /// `imageProvider(index, maxDimension)` loads a source image, optionally
    /// downsampled; images are requested one at a time so full-resolution
    /// sources never need to be resident together.
    public static func compose(cameras: [Int: Camera],
                               meshes: [Int: WarpMesh],
                               options: Options = Options(),
                               imageProvider: (Int, Int?) throws -> RGBImage) rethrows -> (Result, PanoGeometry)? {
        let projection = resolveProjection(options.projection, cameras: cameras)
        guard let source = PanoLayerSource(cameras: cameras, meshes: meshes,
                                           outputWidth: options.outputWidth,
                                           projection: projection) else { return nil }
        guard let result = try compose(source: source, options: options,
                                       imageProvider: imageProvider) else { return nil }
        return (result, source.geometry)
    }

    /// The shared compositing pass over any `LayerSource`. Two passes over
    /// the images: a low-resolution one where every layer is resident at once
    /// (gain needs all pairwise overlaps, the graph cut needs all layers),
    /// then a full-resolution one that loads, projects, and blends one image
    /// at a time so peak memory is one full layer plus the blender's pyramid.
    public static func compose<S: LayerSource>(source full: S,
                                               options: Options = Options(),
                                               imageProvider: (Int, Int?) throws -> RGBImage) rethrows -> Result? {
        let low = full.scaled(toWidth: min(options.seamWidth, full.width))
        let indices = full.imageIndices

        // Low-res layers for gain + seams.
        var lowLayers: [ImageLayer] = []
        for idx in indices {
            let img = try imageProvider(idx, options.seamSourceDimension)
            guard let layer = low.project(imageIndex: idx, image: img) else { continue }
            lowLayers.append(layer)
        }
        guard !lowLayers.isEmpty else { return nil }

        // Gains are applied to the low-res layers before seam finding so the
        // cut's intensity differences measure content, not exposure.
        let gains = GainCompensator.solve(layers: lowLayers)
        for i in lowLayers.indices {
            GainCompensator.apply(gain: gains[lowLayers[i].imageIndex] ?? 1, to: &lowLayers[i])
        }

        let labels = SeamFinder.labels(layers: lowLayers, width: low.width, height: low.height,
                                       localityWeight: options.seamLocalityWeight)

        // Full-res blend, one image at a time to bound memory.
        let blender = MultiBandBlender(width: full.width, height: full.height,
                                       levels: options.blendLevels)
        let sx = Double(low.width) / Double(full.width)
        for idx in indices {
            let img = try imageProvider(idx, full.sourceDimension(for: idx))
            guard var layer = full.project(imageIndex: idx, image: img) else { continue }
            GainCompensator.apply(gain: gains[idx] ?? 1, to: &layer)

            // Seam ownership for this layer, sampled from the low-res label
            // map by nearest pixel. The blocky upsampled boundary is fine:
            // the blender's Gaussian weight pyramid smooths it well past the
            // sampling step.
            var seam = ImageF(width: layer.width, height: layer.height)
            let k = Int32(idx)
            for row in 0..<layer.height {
                let ly = min(low.height - 1, max(0, Int((Double(layer.y0 + row) + 0.5) * sx)))
                for col in 0..<layer.width {
                    let lx = min(low.width - 1, max(0, Int((Double(layer.x0 + col) + 0.5) * sx)))
                    if labels[ly * low.width + lx] == k {
                        seam.pixels[row * layer.width + col] = 1
                    }
                }
            }
            blender.add(rgb: layer.rgb, validity: layer.validity, seamMask: seam,
                        x0: layer.x0, y0: layer.y0)
        }

        var image = blender.finalize()
        if options.crop {
            // Only crop when the covered rectangle is a reasonable fraction
            // of the frame; an odd-shaped mosaic is better left whole than
            // reduced to a sliver.
            let rect = largestCoveredRect(coverage: blender.coverage)
            if rect.w > full.width / 4, rect.h > full.height / 4 {
                image = image.cropped(x0: rect.x, y0: rect.y, width: rect.w, height: rect.h)
            }
        }
        return Result(image: image, gains: gains)
    }

    /// Largest axis-aligned rectangle containing only covered pixels
    /// (histogram-of-heights, computed on a decimated mask for speed).
    /// Returns the whole frame if nothing is covered. Result is in coverage
    /// pixels, snapped to the decimation step.
    static func largestCoveredRect(coverage: ImageF) -> (x: Int, y: Int, w: Int, h: Int) {
        // Decimate to about 600 columns: the O(w·h) stack pass on a 13000 px
        // strip would otherwise dominate, and a few-pixel crop error is moot.
        let step = max(1, coverage.width / 600)
        let w = coverage.width / step, h = coverage.height / step
        guard w > 0, h > 0 else { return (0, 0, coverage.width, coverage.height) }

        var heights = [Int](repeating: 0, count: w)
        var best = (area: 0, x: 0, y: 0, w: 0, h: 0)
        for row in 0..<h {
            for col in 0..<w {
                // Covered only if the whole step-block is covered (conservative).
                var covered = true
                outer: for dy in 0..<step {
                    for dx in 0..<step {
                        if coverage[col * step + dx, row * step + dy] <= 0.5 {
                            covered = false
                            break outer
                        }
                    }
                }
                heights[col] = covered ? heights[col] + 1 : 0
            }
            // Largest rectangle in histogram via monotonic stack.
            var stack: [(start: Int, height: Int)] = []
            for col in 0...w {
                let hCur = col < w ? heights[col] : 0
                var start = col
                while let top = stack.last, top.height >= hCur {
                    stack.removeLast()
                    let area = top.height * (col - top.start)
                    if area > best.area {
                        best = (area, top.start, row - top.height + 1, col - top.start, top.height)
                    }
                    start = top.start
                }
                if hCur > 0 { stack.append((start, hCur)) }
            }
        }
        guard best.area > 0 else { return (0, 0, coverage.width, coverage.height) }
        return (best.x * step, best.y * step, best.w * step, best.h * step)
    }
}
