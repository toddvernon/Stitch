import Foundation

// The pipeline orchestrator. Every stage module in StitchCore (Features,
// Matching, Geometry, Compositing) is a pure function over value types; this
// file is the one place that strings them together in order, decides between
// the rotational and strip models, and packages the result for the CLI and
// the app. No I/O policy lives here beyond reading the source files.

/// One finished panorama (rotational) or strip (multi-viewpoint).
public struct StitchedPanorama {
    /// Full-resolution composite, already cropped if the settings asked for it.
    public let image: RGBImage
    /// The photos that made it into this output, sorted by name. Inputs that
    /// were not placed are absent, so the caller can tell what was left out.
    public let sourceURLs: [URL]
    /// Which model produced it; decides how the app labels and names it.
    public let kind: Stitcher.Kind
    /// Angular span for panoramas; 0 for strips, which have no single viewpoint.
    public let horizontalDegrees: Double
    /// Bundle-adjustment RMS (pre-mesh) for panoramas, global strip-solve
    /// RMS for strips; px at registration scale.
    public let alignmentRMS: Double
}

/// High-level orchestration of the full pipeline, shared by the CLI and the
/// app. For one input set the order is:
///
///   1. Load grayscale at registration scale and detect SIFT (Features).
///   2. Recognize groups (Matching): pairwise RANSAC and verification, then
///      connected components. One input folder can yield several groups.
///   3. Per group, align (Geometry): bundle adjustment plus optional mesh
///      refinement for a panorama, the global similarity solve for a strip.
///   4. Composite (Compositing): gain, graph-cut seams, multi-band blend,
///      crop. Only this step touches full resolution, and it reloads the
///      sources in RGB through a provider callback rather than holding them.
///
/// Registration runs on downsampled images (DESIGN.md, Performance); the
/// caps live in `Settings`.
public enum Stitcher {

    /// Which camera model an output came from: a single-viewpoint rotational
    /// panorama, or a multi-viewpoint strip laid on the facade plane
    /// (Agarwala et al. 2006). Strips have no angular span.
    public enum Kind: String, Sendable {
        case panorama
        case strip
    }

    /// What to look for in the input set.
    public enum Mode: String, CaseIterable, Sendable {
        /// Recognize both ways and keep whichever places more images
        /// (ties go to the panorama, unless GPS says the photographer moved).
        case auto
        /// Rotational model only: homography pairs, bundle adjustment.
        case panorama
        /// Facade-plane model only: similarity pairs, global linear solve.
        case strip
    }

    /// Everything the CLI flags and the app pickers can change. Defaults are
    /// the values that produced the reference outputs in STATUS.md.
    public struct Settings {
        public var mode: Mode = .auto
        /// Longer side, in pixels, that sources are downsampled to for
        /// feature detection and alignment. About 2 to 3 MP is plenty for
        /// SIFT and keeps registration fast; rendering ignores this cap.
        public var registrationMaxDimension = 2000
        /// Strips live on a thin band of distant facade: they need more
        /// registration resolution than a panorama to find the same matches.
        public var stripRegistrationMaxDimension = 3000
        /// 0 = natural full resolution, capped at `maxOutputWidth`.
        public var outputWidth = 0
        /// Ceiling on the auto-chosen width. The blender holds several float
        /// pyramids of the whole output at once, so memory scales with this
        /// (streaming the blend is an open item in DESIGN.md).
        public var maxOutputWidth = 12000
        /// Strips are long by nature; their natural width gets more room.
        public var maxStripWidth = 20000
        /// Run the parallax mesh stage (pipeline stage 7). Off only for
        /// diagnosing whether a defect comes from the rigid alignment.
        public var useMesh = true
        /// Crop to the largest fully covered rectangle; off keeps the whole
        /// projected footprint with transparent gaps.
        public var crop = true
        /// nil = auto: Pannini under 160° of span, spherical above.
        public var projection: PanoProjection? = nil
        /// Seam preference for the nearest-center photo in strips.
        public var stripSeamLocality = 0.01
        /// Strips' exposure differences are local (sun angle changes as you
        /// walk) and gain compensation can't remove them; wider blend bands
        /// spread what remains over hundreds of pixels.
        public var stripBlendLevels = 8
        public init() {}
    }

    /// Runs the whole pipeline on a set of files and returns one output per
    /// recognized group, in recognition order (largest group first). Inputs
    /// that place in no group are reported through `progress` and dropped.
    /// `progress` receives human-readable log lines and is called on the
    /// caller's thread; the app hops them to the main actor itself.
    public static func stitch(urls: [URL],
                              settings: Settings = Settings(),
                              progress: (String) -> Void = { _ in }) throws -> [StitchedPanorama] {
        guard urls.count >= 2 else { return [] }

        let detector = SIFTDetector()
        // Detection is a closure because auto mode may need to run it twice
        // (see below); grayscale is all SIFT wants, RGB waits for compositing.
        func detect(maxDimension: Int) throws -> (images: [ImageF], features: [[Feature]]) {
            var images: [ImageF] = []
            var features: [[Feature]] = []
            for url in urls {
                let img = try ImageLoader.loadGrayscale(url: url, maxDimension: maxDimension)
                let f = detector.detect(in: img)
                progress("\(url.lastPathComponent): \(img.width)x\(img.height), \(f.count) features")
                images.append(img)
                features.append(f)
            }
            return (images, features)
        }
        // Positions are advisory only: a log line, an explanation for photos
        // that didn't connect, and a tiebreak. Missing GPS changes nothing.
        let coords = GPSHints.coordinates(urls: urls)
        if let line = GPSHints.summary(coords, totalPhotos: urls.count) { progress(line) }
        let names = urls.map(\.lastPathComponent)

        // The strip cap never drops below the panorama cap, so an explicit
        // --max-dim larger than 3000 applies to strips too. An explicit strip
        // request detects at strip resolution straight away; auto starts at
        // panorama resolution because that is the common case and cheaper.
        let stripDimension = max(settings.registrationMaxDimension, settings.stripRegistrationMaxDimension)
        var (images, features) = try detect(maxDimension: settings.mode == .strip
                                            ? stripDimension : settings.registrationMaxDimension)
        var sizes = images.map { (width: $0.width, height: $0.height) }

        var (kind, groups) = recognize(features: features, sizes: sizes, mode: settings.mode,
                                       gpsSpread: GPSHints.spread(coords), progress: progress)
        if kind == .strip, settings.mode == .auto, stripDimension > settings.registrationMaxDimension {
            // Auto chose a strip on panorama-resolution features; redo
            // detection at strip resolution, which places more images
            // (BeachWalk chained 3 of 5 at 2000 px and all 5 at 3000).
            // The decision is already made, so the second pass is strip-only
            // and needs no GPS tiebreak.
            progress("re-detecting at \(stripDimension) px for the strip…")
            (images, features) = try detect(maxDimension: stripDimension)
            sizes = images.map { (width: $0.width, height: $0.height) }
            (kind, groups) = recognize(features: features, sizes: sizes, mode: .strip,
                                       gpsSpread: nil, progress: progress)
        }
        // Report what did not connect, and let GPS explain it where it can
        // (a photo far beyond the typical step is a dropped frame, not a bug).
        let placed = Set(groups.flatMap(\.imageIndices))
        let unplaced = (0..<urls.count).filter { !placed.contains($0) }
        if !unplaced.isEmpty, !groups.isEmpty {
            progress("unmatched: \(unplaced.map { names[$0] }.joined(separator: ", "))")
        }
        for note in GPSHints.gapNotes(unplaced: unplaced, coords: coords, names: names) {
            progress(note)
        }
        guard !groups.isEmpty else {
            progress("no \(kind == .strip ? "strips" : "panoramas") recognized")
            return []
        }
        let noun = kind == .strip ? "strip" : "panorama"
        progress("recognized \(groups.count) \(noun)\(groups.count == 1 ? "" : "s")")

        // Groups are independent from here on: each gets its own alignment
        // and render. Failures skip the group rather than aborting the set.
        var results: [StitchedPanorama] = []
        for (g, group) in groups.enumerated() {
            let tag = groups.count > 1 ? "\(noun) \(g + 1): " : ""
            let result: StitchedPanorama?
            switch kind {
            case .panorama:
                result = try stitchPanorama(group: group, urls: urls, images: images, features: features,
                                            sizes: sizes, settings: settings, tag: tag, progress: progress)
            case .strip:
                result = try stitchStrip(group: group, urls: urls, features: features,
                                         sizes: sizes, settings: settings, tag: tag, progress: progress)
            }
            if let result { results.append(result) }
        }
        return results
    }

    /// Runs recognition per the mode. Auto tries both models and keeps the
    /// one whose largest group holds more images: a rotational set verifies
    /// as a panorama easily and a walked set never does, so the choice is
    /// rarely close. A tie falls to the panorama unless GPS says the
    /// photographer moved (`gpsSpread` beyond `GPSHints.movedThreshold`).
    static func recognize(features: [[Feature]], sizes: [(width: Int, height: Int)],
                          mode: Mode, gpsSpread: Double? = nil,
                          progress: (String) -> Void) -> (Kind, [PanoramaGroup]) {
        switch mode {
        case .panorama:
            return (.panorama, PanoramaRecognizer.recognize(features: features, imageSizes: sizes))
        case .strip:
            return (.strip, PanoramaRecognizer.recognize(features: features, imageSizes: sizes,
                                                         model: .similarity))
        case .auto:
            let pano = PanoramaRecognizer.recognize(features: features, imageSizes: sizes)
            let strip = PanoramaRecognizer.recognize(features: features, imageSizes: sizes,
                                                     model: .similarity)
            let panoBest = pano.first?.imageIndices.count ?? 0
            let stripBest = strip.first?.imageIndices.count ?? 0
            if stripBest > panoBest {
                progress("auto: strip (\(stripBest) images placed vs \(panoBest) as a panorama)")
                return (.strip, strip)
            }
            if stripBest == panoBest, let spread = gpsSpread, spread >= GPSHints.movedThreshold {
                progress("auto: strip (tie at \(stripBest) images; GPS says the photos span \(Int(spread.rounded())) m)")
                return (.strip, strip)
            }
            return (.panorama, pano)
        }
    }

    /// The rotational path for one recognized group: bundle adjustment and
    /// straightening, mesh refinement, projection choice, output sizing,
    /// then the composite. Returns nil (after logging why) if alignment or
    /// compositing fails, so the caller can move on to the next group.
    static func stitchPanorama(group: PanoramaGroup, urls: [URL], images: [ImageF],
                               features: [[Feature]], sizes: [(width: Int, height: Int)],
                               settings: Settings, tag: String,
                               progress: (String) -> Void) throws -> StitchedPanorama? {
        progress("\(tag)aligning \(group.imageIndices.count) images…")

        // EXIF 35 mm-equivalent focal length seeds the bundle adjuster
        // (paper §4). A 35 mm frame is 36 mm wide, so f_px = f35 / 36 × the
        // long side at registration scale. Missing EXIF falls back to the
        // aligner's own estimate; the hint only shortens convergence.
        var focalHints: [Int: Double] = [:]
        for idx in group.imageIndices {
            if let f35 = ImageLoader.focalLength35mm(url: urls[idx]) {
                focalHints[idx] = f35 / 36.0 * Double(max(images[idx].width, images[idx].height))
            }
        }
        guard let alignment = PanoramaAligner.align(group: group, features: features,
                                                    imageSizes: sizes, focalHints: focalHints) else {
            progress("\(tag)alignment failed, skipping")
            return nil
        }
        progress("\(tag)bundle adjustment RMS \(String(format: "%.2f", alignment.finalRMS)) px")

        // Stage 7: per-image warp meshes close the parallax gap the rigid
        // rotation model cannot. When residuals are already small the solved
        // offsets are near zero, so tripod sets stay effectively rigid.
        var meshes: [Int: WarpMesh] = [:]
        if settings.useMesh {
            let refined = MeshRefiner.refine(group: group, features: features,
                                             cameras: alignment.cameras)
            meshes = refined.meshes
            progress("\(tag)parallax residual \(String(format: "%.2f", refined.initialRMS)) → \(String(format: "%.2f", refined.finalRMS)) px")
        }

        // nil projection = auto: Pannini under 160° of span, spherical above
        // (DESIGN.md, Output projections). Resolved here so the log and the
        // width estimate both see the projection actually used.
        let projection = Compositor.resolveProjection(settings.projection,
                                                      cameras: alignment.cameras)
        progress("\(tag)projection: \(projection.rawValue)")

        // Natural width keeps center resolution close to the sources'; the
        // cap protects the blender's memory on very wide spans.
        var width = settings.outputWidth
        if width == 0 {
            width = min(naturalWidth(cameras: alignment.cameras, urls: urls,
                                     projection: projection),
                        settings.maxOutputWidth)
            progress("\(tag)output width \(width) px")
        }

        var options = Compositor.Options()
        options.outputWidth = width
        options.crop = settings.crop
        options.projection = projection
        progress("\(tag)compositing…")
        guard let (composed, geometry) = try Compositor.compose(cameras: alignment.cameras,
                                                                meshes: meshes,
                                                                options: options,
                                                                imageProvider: { idx, maxDim in
            try ImageLoader.loadRGB(url: urls[idx], maxDimension: maxDim)
        }) else {
            progress("\(tag)compositing failed, skipping")
            return nil
        }
        let degrees = (geometry.thetaMax - geometry.thetaMin) * 180 / .pi
        progress("\(tag)rendered \(composed.image.width)x\(composed.image.height) (\(String(format: "%.0f", degrees))° span)")
        return StitchedPanorama(image: composed.image,
                                sourceURLs: group.imageIndices.sorted().map { urls[$0] },
                                kind: .panorama,
                                horizontalDegrees: degrees,
                                alignmentRMS: alignment.finalRMS)
    }

    /// The strip path for one recognized group: global similarity solve,
    /// planar layer source, output sizing, then the same compositor as the
    /// panorama path with the strip-specific seam and blend settings. No
    /// mesh stage: the strip model has no single viewpoint to refine toward.
    static func stitchStrip(group: PanoramaGroup, urls: [URL],
                            features: [[Feature]], sizes: [(width: Int, height: Int)],
                            settings: Settings, tag: String,
                            progress: (String) -> Void) throws -> StitchedPanorama? {
        progress("\(tag)placing \(group.imageIndices.count) images on the facade plane…")
        guard let alignment = StripAligner.align(group: group, features: features) else {
            progress("\(tag)alignment failed, skipping")
            return nil
        }
        progress("\(tag)strip solve RMS \(String(format: "%.2f", alignment.finalRMS)) px")

        // StripGeometry wants sizes keyed by image index, for only the images
        // in this group; `sizes` is positional over the whole input set.
        var sizeMap: [Int: (width: Int, height: Int)] = [:]
        for idx in group.imageIndices { sizeMap[idx] = sizes[idx] }

        var width = settings.outputWidth
        if width == 0 {
            width = min(naturalStripWidth(transforms: alignment.transforms, sizes: sizeMap, urls: urls),
                        settings.maxStripWidth)
            progress("\(tag)output width \(width) px")
        }
        guard let source = StripGeometry(transforms: alignment.transforms, sizes: sizeMap,
                                         outputWidth: width) else {
            progress("\(tag)empty strip, skipping")
            return nil
        }

        // Strip-specific compositor settings: the viewpoint-locality term
        // is what makes the seam finder pick the most head-on photo per
        // region (DESIGN.md, strip step 5); the deeper blend pyramid hides
        // the exposure drift gain compensation cannot remove (step 6).
        var options = Compositor.Options()
        options.outputWidth = width
        options.crop = settings.crop
        options.seamLocalityWeight = settings.stripSeamLocality
        options.blendLevels = settings.stripBlendLevels
        // Strips are wide and short: size the seam pass by area (what the
        // graph cut's cost depends on), keeping the pixel budget of a 16:9
        // pass at the default width.
        let aspect = Double(source.width) / Double(max(1, source.height))
        let budget = Double(options.seamWidth * options.seamWidth) * 9 / 16
        options.seamWidth = max(options.seamWidth, min(2500, Int((budget * aspect).squareRoot())))
        progress("\(tag)compositing…")
        guard let composed = try Compositor.compose(source: source, options: options,
                                                    imageProvider: { idx, maxDim in
            try ImageLoader.loadRGB(url: urls[idx], maxDimension: maxDim)
        }) else {
            progress("\(tag)compositing failed, skipping")
            return nil
        }
        progress("\(tag)rendered \(composed.image.width)x\(composed.image.height)")
        return StitchedPanorama(image: composed.image,
                                sourceURLs: group.imageIndices.sorted().map { urls[$0] },
                                kind: .strip,
                                horizontalDegrees: 0,
                                alignmentRMS: alignment.finalRMS)
    }

    /// Projection span × mean focal at the sources' native resolution
    /// (u ≈ θ at the pano center, so this keeps center resolution ≈ source).
    /// The 1000 px probe geometry exists only to measure the projected span;
    /// its width is arbitrary because `uSpan` is in projection units.
    /// 4000 is the fallback when the geometry or file headers are unusable.
    static func naturalWidth(cameras: [Int: Camera], urls: [URL],
                             projection: PanoProjection = .spherical) -> Int {
        guard let probe = PanoGeometry(cameras: cameras, outputWidth: 1000,
                                       projection: projection) else { return 4000 }
        var scaled: [Double] = []
        for (idx, cam) in cameras {
            guard idx < urls.count, let dims = ImageLoader.pixelDimensions(url: urls[idx]) else { continue }
            let fullLong = Double(max(dims.width, dims.height))
            let regLong = Double(max(cam.width, cam.height))
            scaled.append(cam.focal * fullLong / regLong)
        }
        guard !scaled.isEmpty else { return 4000 }
        let fMean = scaled.reduce(0, +) / Double(scaled.count)
        return max(1000, Int(probe.uSpan * fMean))
    }

    /// Strip extent × mean native pixels per strip unit, so the output
    /// resolution matches the sources'. Each image's similarity scale maps
    /// registration pixels to strip units; dividing it out of the native-to-
    /// registration ratio gives native pixels per strip unit.
    static func naturalStripWidth(transforms: [Int: Similarity],
                                  sizes: [Int: (width: Int, height: Int)], urls: [URL]) -> Int {
        guard let probe = StripGeometry(transforms: transforms, sizes: sizes, outputWidth: 1000)
        else { return 4000 }
        var perUnit: [Double] = []
        for (idx, t) in transforms {
            guard idx < urls.count, let size = sizes[idx],
                  let dims = ImageLoader.pixelDimensions(url: urls[idx]) else { continue }
            let fullLong = Double(max(dims.width, dims.height))
            let regLong = Double(max(size.width, size.height))
            perUnit.append(fullLong / regLong / t.scale)
        }
        guard !perUnit.isEmpty else { return 4000 }
        let mean = perUnit.reduce(0, +) / Double(perUnit.count)
        return max(1000, Int(probe.extent.x * mean))
    }

    /// Image files in a folder (or the URLs themselves if already files),
    /// sorted by name. This is the one input-resolution rule the CLI and the
    /// app share, so a folder means the same set of photos in both. Earlier
    /// stitch outputs sitting in the folder (recognized by the software tag
    /// the writer stamps into the metadata) are skipped so a re-run does not
    /// feed the last result back in.
    public static func imageURLs(from dropped: [URL]) -> [URL] {
        let extensions = Set(["jpg", "jpeg", "png", "heic", "tif", "tiff"])
        var files: [URL] = []
        for url in dropped {
            var isDir: ObjCBool = false
            if FileManager.default.fileExists(atPath: url.path, isDirectory: &isDir), isDir.boolValue {
                let contents = (try? FileManager.default.contentsOfDirectory(
                    at: url, includingPropertiesForKeys: nil)) ?? []
                files.append(contentsOf: contents.filter { extensions.contains($0.pathExtension.lowercased()) })
            } else if extensions.contains(url.pathExtension.lowercased()) {
                files.append(url)
            }
        }
        return files
            .filter { !ImageLoader.isStitchOutput(url: $0) }
            .sorted { $0.lastPathComponent < $1.lastPathComponent }
    }
}
