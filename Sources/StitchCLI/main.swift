import Foundation
import StitchCore

func usage(exitCode: Int32 = 64) -> Never {
    print("""
    usage: stitch <command> [options]

    commands:
      features <image> [--debug-out <path.png>] [--max-dim <N>] [--double]
          Detect SIFT features in an image. Prints a summary; optionally writes
          a PNG with keypoints overlaid.

      match <imageA> <imageB> [--debug-out <path.png>] [--max-dim <N>]
          Match SIFT features between two images, estimate the homography with
          RANSAC, and run pair verification. Optionally writes a side-by-side
          PNG with inlier (green) and outlier (red) correspondences.

      recognize <folder> [--max-dim <N>]
          Detect features in every image in a folder and report the recognized
          panoramas (connected components of verified pairs).

      pano <folder> -o <out.png|jpg|tiff> [--max-dim <N>] [--width <W>]
                    [--no-mesh] [--no-crop] [--projection <p>]
          --projection: spherical, cylindrical, pannini, or auto (default:
          pannini under 160° of span for a natural perspective look,
          spherical above).
          Full pipeline: recognize, bundle adjust, straighten, refine parallax
          with warp meshes (skip with --no-mesh), then composite with gain
          compensation, graph-cut seams, and multi-band blending, cropped to
          the largest covered rectangle (--no-crop keeps the full sphere
          projection). Every recognized panorama is written (extras suffixed
          .2, .3, …). Output width defaults to the panorama's natural full
          resolution (capped at 12000 px); override with --width.

    options:
      --max-dim <N>    downsample so the longer side is at most N pixels (default 2000)
      --double         double the image before detection (more features, 4x slower)
      --debug-out <p>  write keypoint overlay PNG to <p>
    """)
    exit(exitCode)
}

func runFeatures(_ args: [String]) throws {
    var inputPath: String?
    var debugOut: String?
    var maxDim = 2000
    var doubleImage = false

    var it = args.makeIterator()
    while let arg = it.next() {
        switch arg {
        case "--debug-out":
            guard let v = it.next() else { usage() }
            debugOut = v
        case "--max-dim":
            guard let v = it.next(), let n = Int(v), n > 0 else { usage() }
            maxDim = n
        case "--double":
            doubleImage = true
        default:
            if arg.hasPrefix("-") || inputPath != nil { usage() }
            inputPath = arg
        }
    }
    guard let inputPath else { usage() }
    let url = URL(fileURLWithPath: inputPath)

    let loadStart = Date()
    let image = try ImageLoader.loadGrayscale(url: url, maxDimension: maxDim)
    let loadTime = Date().timeIntervalSince(loadStart)

    var config = SIFTConfig()
    config.doubleImage = doubleImage
    let detector = SIFTDetector(config: config)

    let detectStart = Date()
    let features = detector.detect(in: image)
    let detectTime = Date().timeIntervalSince(detectStart)

    print("image: \(url.lastPathComponent)  \(image.width)x\(image.height)  (loaded in \(String(format: "%.2f", loadTime))s)")
    print("features: \(features.count)  (detected in \(String(format: "%.2f", detectTime))s)")
    if let f35 = ImageLoader.focalLength35mm(url: url) {
        print("EXIF focal length (35mm equiv): \(f35)mm")
    }
    if !features.isEmpty {
        let scales = features.map(\.scale).sorted()
        print("scale range: \(String(format: "%.1f", scales.first!)) – \(String(format: "%.1f", scales.last!)) px")
    }

    if let debugOut {
        let overlay = DebugRender.featureOverlay(image: image, features: features)
        let outURL = URL(fileURLWithPath: debugOut)
        try ImageLoader.writePNG(overlay, to: outURL)
        print("debug overlay: \(outURL.path)")
    }
}

func runMatch(_ args: [String]) throws {
    var inputPaths: [String] = []
    var debugOut: String?
    var maxDim = 2000

    var it = args.makeIterator()
    while let arg = it.next() {
        switch arg {
        case "--debug-out":
            guard let v = it.next() else { usage() }
            debugOut = v
        case "--max-dim":
            guard let v = it.next(), let n = Int(v), n > 0 else { usage() }
            maxDim = n
        default:
            if arg.hasPrefix("-") || inputPaths.count >= 2 { usage() }
            inputPaths.append(arg)
        }
    }
    guard inputPaths.count == 2 else { usage() }
    let urlA = URL(fileURLWithPath: inputPaths[0])
    let urlB = URL(fileURLWithPath: inputPaths[1])

    let imageA = try ImageLoader.loadGrayscale(url: urlA, maxDimension: maxDim)
    let imageB = try ImageLoader.loadGrayscale(url: urlB, maxDimension: maxDim)

    let detector = SIFTDetector()
    let start = Date()
    let featuresA = detector.detect(in: imageA)
    let featuresB = detector.detect(in: imageB)
    let detectTime = Date().timeIntervalSince(start)

    let matchStart = Date()
    let matches = DescriptorMatcher.match(featuresA, featuresB)
    let matchTime = Date().timeIntervalSince(matchStart)

    print("\(urlA.lastPathComponent): \(featuresA.count) features, \(urlB.lastPathComponent): \(featuresB.count) features (\(String(format: "%.2f", detectTime))s)")
    print("putative matches: \(matches.count) (\(String(format: "%.2f", matchTime))s)")

    guard let geometry = PairEstimator.estimate(featuresA: featuresA, featuresB: featuresB,
                                                matches: matches,
                                                imageBWidth: imageB.width,
                                                imageBHeight: imageB.height) else {
        print("no homography could be estimated — images likely do not overlap")
        return
    }

    let ni = geometry.inlierIndices.count
    let nf = geometry.overlapMatchCount
    let threshold = PairEstimator.verificationAlpha + PairEstimator.verificationBeta * Double(nf)
    print("RANSAC inliers: \(ni) of \(matches.count), matches in overlap: \(nf)")
    print("verification: n_i=\(ni) \(geometry.isVerified ? ">" : "<=") \(String(format: "%.1f", threshold))  →  \(geometry.isVerified ? "MATCH" : "NO MATCH")")

    let h = geometry.homography
    for r in 0..<3 {
        print(String(format: "  H[%d] = [%10.5f %10.5f %10.5f]", r, h[0][r], h[1][r], h[2][r]))
    }

    if let debugOut {
        let overlay = DebugRender.matchOverlay(imageA: imageA, imageB: imageB,
                                               featuresA: featuresA, featuresB: featuresB,
                                               matches: matches, inlierIndices: geometry.inlierIndices)
        let outURL = URL(fileURLWithPath: debugOut)
        try ImageLoader.writePNG(overlay, to: outURL)
        print("debug overlay: \(outURL.path)")
    }
}

/// Shared front half of recognize/pano: load, detect, recognize.
func recognizePanoramas(folder: String, maxDim: Int) throws
    -> (urls: [URL], images: [ImageF], features: [[Feature]], groups: [PanoramaGroup]) {
    let urls = Stitcher.imageURLs(from: [URL(fileURLWithPath: folder)])
    guard urls.count >= 2 else {
        print("need at least 2 images in \(folder)")
        exit(1)
    }
    var images: [ImageF] = []
    var features: [[Feature]] = []
    let detector = SIFTDetector()
    for url in urls {
        let img = try ImageLoader.loadGrayscale(url: url, maxDimension: maxDim)
        let f = detector.detect(in: img)
        print("  \(url.lastPathComponent): \(img.width)x\(img.height), \(f.count) features")
        images.append(img)
        features.append(f)
    }
    let sizes = images.map { (width: $0.width, height: $0.height) }
    let start = Date()
    let groups = PanoramaRecognizer.recognize(features: features, imageSizes: sizes)
    print("recognition: \(String(format: "%.2f", Date().timeIntervalSince(start)))s")
    return (urls, images, features, groups)
}

func runRecognize(_ args: [String]) throws {
    var folder: String?
    var maxDim = 2000
    var it = args.makeIterator()
    while let arg = it.next() {
        switch arg {
        case "--max-dim":
            guard let v = it.next(), let n = Int(v), n > 0 else { usage() }
            maxDim = n
        default:
            if arg.hasPrefix("-") || folder != nil { usage() }
            folder = arg
        }
    }
    guard let folder else { usage() }
    let (urls, _, _, groups) = try recognizePanoramas(folder: folder, maxDim: maxDim)

    if groups.isEmpty {
        print("no panoramas recognized")
        return
    }
    for (i, group) in groups.enumerated() {
        let names = group.imageIndices.sorted().map { urls[$0].lastPathComponent }.joined(separator: ", ")
        print("panorama \(i + 1): \(group.imageIndices.count) images (\(names))")
        for p in group.pairs {
            print("    \(urls[p.indexA].lastPathComponent) <-> \(urls[p.indexB].lastPathComponent): \(p.geometry.inlierIndices.count) inliers")
        }
    }
    let matched = Set(groups.flatMap(\.imageIndices))
    let noise = (0..<urls.count).filter { !matched.contains($0) }
    if !noise.isEmpty {
        print("unmatched: \(noise.map { urls[$0].lastPathComponent }.joined(separator: ", "))")
    }
}

func runPano(_ args: [String]) throws {
    var folder: String?
    var outPath: String?
    var settings = Stitcher.Settings()
    var it = args.makeIterator()
    while let arg = it.next() {
        switch arg {
        case "-o", "--out":
            guard let v = it.next() else { usage() }
            outPath = v
        case "--max-dim":
            guard let v = it.next(), let n = Int(v), n > 0 else { usage() }
            settings.registrationMaxDimension = n
        case "--width":
            guard let v = it.next(), let n = Int(v), n > 0 else { usage() }
            settings.outputWidth = n
        case "--no-mesh":
            settings.useMesh = false
        case "--no-crop":
            settings.crop = false
        case "--projection":
            guard let v = it.next() else { usage() }
            if v == "auto" {
                settings.projection = nil
            } else if let p = PanoProjection(rawValue: v) {
                settings.projection = p
            } else {
                usage()
            }
        default:
            if arg.hasPrefix("-") || folder != nil { usage() }
            folder = arg
        }
    }
    guard let folder, let outPath else { usage() }

    let urls = Stitcher.imageURLs(from: [URL(fileURLWithPath: folder)])
    guard urls.count >= 2 else {
        print("need at least 2 images in \(folder)")
        exit(1)
    }
    let start = Date()
    let panoramas = try Stitcher.stitch(urls: urls, settings: settings) { print($0) }
    guard !panoramas.isEmpty else {
        print("no panorama produced")
        exit(1)
    }
    print("total: \(String(format: "%.1f", Date().timeIntervalSince(start)))s")

    let outURL = URL(fileURLWithPath: outPath)
    for (i, pano) in panoramas.enumerated() {
        // First panorama gets the requested name; extras get -2, -3, …
        let url: URL
        if i == 0 {
            url = outURL
        } else {
            let base = outURL.deletingPathExtension()
            url = base.appendingPathExtension("\(i + 1).\(outURL.pathExtension)")
        }
        try ImageLoader.writeImage(pano.image.makeCGImage(), to: url)
        print("panorama: \(url.path)")
    }
}

let arguments = Array(CommandLine.arguments.dropFirst())
guard let command = arguments.first else { usage() }

do {
    switch command {
    case "features":
        try runFeatures(Array(arguments.dropFirst()))
    case "match":
        try runMatch(Array(arguments.dropFirst()))
    case "recognize":
        try runRecognize(Array(arguments.dropFirst()))
    case "pano":
        try runPano(Array(arguments.dropFirst()))
    case "help", "-h", "--help":
        usage(exitCode: 0)
    default:
        usage()
    }
} catch {
    FileHandle.standardError.write("error: \(error)\n".data(using: .utf8)!)
    exit(1)
}
