import Foundation

/// Milestone-6 compositing pipeline: project layers, compensate gain, find
/// graph-cut seams at working resolution, multi-band blend at full resolution,
/// and crop to the largest fully-covered rectangle.
public enum Compositor {

    public struct Options {
        public var outputWidth = 4000
        /// Working width for gain/seam estimation. Kept small deliberately:
        /// graph-cut cost grows steeply with node count, and seam placement
        /// error of a few working pixels disappears under multi-band blending.
        public var seamWidth = 700
        /// Long-side cap when loading sources for the low-res gain/seam pass.
        public var seamSourceDimension = 800
        public var blendLevels = 5
        public var crop = true
        public init() {}
    }

    public struct Result {
        public var image: RGBImage
        public var gains: [Int: Double]
        public var geometry: PanoGeometry
    }

    /// `imageProvider(index, maxDimension)` loads a source image, optionally
    /// downsampled; images are requested one at a time so full-resolution
    /// sources never need to be resident together.
    public static func compose(cameras: [Int: Camera],
                               meshes: [Int: WarpMesh],
                               options: Options = Options(),
                               imageProvider: (Int, Int?) throws -> RGBImage) rethrows -> Result? {
        guard let geoFull = PanoGeometry(cameras: cameras, outputWidth: options.outputWidth) else { return nil }
        let geoLow = geoFull.scaled(toWidth: min(options.seamWidth, options.outputWidth))
        let indices = cameras.keys.sorted()

        // Low-res layers for gain + seams.
        var lowLayers: [ImageLayer] = []
        for idx in indices {
            let img = try imageProvider(idx, options.seamSourceDimension)
            guard let layer = LayerProjector.project(imageIndex: idx, camera: cameras[idx]!,
                                                     image: img, mesh: meshes[idx],
                                                     geometry: geoLow) else { continue }
            lowLayers.append(layer)
        }
        guard !lowLayers.isEmpty else { return nil }

        let gains = GainCompensator.solve(layers: lowLayers)
        for i in lowLayers.indices {
            GainCompensator.apply(gain: gains[lowLayers[i].imageIndex] ?? 1, to: &lowLayers[i])
        }

        let labels = SeamFinder.labels(layers: lowLayers, width: geoLow.width, height: geoLow.height)

        // Full-res blend, one image at a time to bound memory.
        let blender = MultiBandBlender(width: geoFull.width, height: geoFull.height,
                                       levels: options.blendLevels)
        let sx = Double(geoLow.width) / Double(geoFull.width)
        for idx in indices {
            // Load each source no larger than the output resolution demands:
            // pano px/rad divided by the camera's px/rad, with sampling margin.
            let cam = cameras[idx]!
            let needed = Double(max(cam.width, cam.height)) * geoFull.scale / cam.focal * 1.2
            let img = try imageProvider(idx, Int(needed.rounded(.up)))
            guard var layer = LayerProjector.project(imageIndex: idx, camera: cameras[idx]!,
                                                     image: img, mesh: meshes[idx],
                                                     geometry: geoFull) else { continue }
            GainCompensator.apply(gain: gains[idx] ?? 1, to: &layer)

            // Seam ownership for this layer, sampled from the low-res label map.
            var seam = ImageF(width: layer.width, height: layer.height)
            let k = Int32(idx)
            for row in 0..<layer.height {
                let ly = min(geoLow.height - 1, max(0, Int((Double(layer.y0 + row) + 0.5) * sx)))
                for col in 0..<layer.width {
                    let lx = min(geoLow.width - 1, max(0, Int((Double(layer.x0 + col) + 0.5) * sx)))
                    if labels[ly * geoLow.width + lx] == k {
                        seam.pixels[row * layer.width + col] = 1
                    }
                }
            }
            blender.add(rgb: layer.rgb, validity: layer.validity, seamMask: seam,
                        x0: layer.x0, y0: layer.y0)
        }

        var image = blender.finalize()
        if options.crop {
            let rect = largestCoveredRect(coverage: blender.coverage)
            if rect.w > geoFull.width / 4, rect.h > geoFull.height / 4 {
                image = image.cropped(x0: rect.x, y0: rect.y, width: rect.w, height: rect.h)
            }
        }
        return Result(image: image, gains: gains, geometry: geoFull)
    }

    /// Largest axis-aligned rectangle containing only covered pixels
    /// (histogram-of-heights, computed on a decimated mask for speed).
    static func largestCoveredRect(coverage: ImageF) -> (x: Int, y: Int, w: Int, h: Int) {
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
