import Foundation

/// One finished panorama.
public struct StitchedPanorama {
    public let image: RGBImage
    public let sourceURLs: [URL]
    public let horizontalDegrees: Double
    /// Bundle-adjustment RMS (pre-mesh), px at registration scale.
    public let alignmentRMS: Double
}

/// High-level orchestration of the full pipeline, shared by the CLI and the
/// app: load, detect, recognize, align, refine, composite — for every
/// recognized panorama in the input set.
public enum Stitcher {

    public struct Settings {
        public var registrationMaxDimension = 2000
        /// 0 = natural full resolution, capped at `maxOutputWidth`.
        public var outputWidth = 0
        public var maxOutputWidth = 12000
        public var useMesh = true
        public var crop = true
        /// nil = auto: Pannini under 160° of span, spherical above.
        public var projection: PanoProjection? = nil
        public init() {}
    }

    public static func stitch(urls: [URL],
                              settings: Settings = Settings(),
                              progress: (String) -> Void = { _ in }) throws -> [StitchedPanorama] {
        guard urls.count >= 2 else { return [] }

        var images: [ImageF] = []
        var features: [[Feature]] = []
        let detector = SIFTDetector()
        for url in urls {
            let img = try ImageLoader.loadGrayscale(url: url, maxDimension: settings.registrationMaxDimension)
            let f = detector.detect(in: img)
            progress("\(url.lastPathComponent): \(img.width)x\(img.height), \(f.count) features")
            images.append(img)
            features.append(f)
        }
        let sizes = images.map { (width: $0.width, height: $0.height) }

        let groups = PanoramaRecognizer.recognize(features: features, imageSizes: sizes)
        guard !groups.isEmpty else {
            progress("no panoramas recognized")
            return []
        }
        progress("recognized \(groups.count) panorama\(groups.count == 1 ? "" : "s")")

        var results: [StitchedPanorama] = []
        for (g, group) in groups.enumerated() {
            let tag = groups.count > 1 ? "panorama \(g + 1): " : ""
            progress("\(tag)aligning \(group.imageIndices.count) images…")

            var focalHints: [Int: Double] = [:]
            for idx in group.imageIndices {
                if let f35 = ImageLoader.focalLength35mm(url: urls[idx]) {
                    focalHints[idx] = f35 / 36.0 * Double(max(images[idx].width, images[idx].height))
                }
            }
            guard let alignment = PanoramaAligner.align(group: group, features: features,
                                                        imageSizes: sizes, focalHints: focalHints) else {
                progress("\(tag)alignment failed, skipping")
                continue
            }
            progress("\(tag)bundle adjustment RMS \(String(format: "%.2f", alignment.finalRMS)) px")

            var meshes: [Int: WarpMesh] = [:]
            if settings.useMesh {
                let refined = MeshRefiner.refine(group: group, features: features,
                                                 cameras: alignment.cameras)
                meshes = refined.meshes
                progress("\(tag)parallax residual \(String(format: "%.2f", refined.initialRMS)) → \(String(format: "%.2f", refined.finalRMS)) px")
            }

            let projection = Compositor.resolveProjection(settings.projection,
                                                          cameras: alignment.cameras)
            progress("\(tag)projection: \(projection.rawValue)")

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
            guard let composed = try Compositor.compose(cameras: alignment.cameras,
                                                        meshes: meshes,
                                                        options: options,
                                                        imageProvider: { idx, maxDim in
                try ImageLoader.loadRGB(url: urls[idx], maxDimension: maxDim)
            }) else {
                progress("\(tag)compositing failed, skipping")
                continue
            }
            let degrees = (composed.geometry.thetaMax - composed.geometry.thetaMin) * 180 / .pi
            progress("\(tag)rendered \(composed.image.width)x\(composed.image.height) (\(String(format: "%.0f", degrees))° span)")
            results.append(StitchedPanorama(image: composed.image,
                                            sourceURLs: group.imageIndices.sorted().map { urls[$0] },
                                            horizontalDegrees: degrees,
                                            alignmentRMS: alignment.finalRMS))
        }
        return results
    }

    /// Projection span × mean focal at the sources' native resolution
    /// (u ≈ θ at the pano center, so this keeps center resolution ≈ source).
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

    /// Image files in a folder (or the URLs themselves if already files),
    /// sorted by name — the shared input-resolution rule.
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
        return files.sorted { $0.lastPathComponent < $1.lastPathComponent }
    }
}
