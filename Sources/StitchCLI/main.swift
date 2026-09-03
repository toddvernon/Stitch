import Foundation
import StitchCore

func usage() -> Never {
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

    options:
      --max-dim <N>    downsample so the longer side is at most N pixels (default 2000)
      --double         double the image before detection (more features, 4x slower)
      --debug-out <p>  write keypoint overlay PNG to <p>
    """)
    exit(64)
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

let arguments = Array(CommandLine.arguments.dropFirst())
guard let command = arguments.first else { usage() }

do {
    switch command {
    case "features":
        try runFeatures(Array(arguments.dropFirst()))
    case "match":
        try runMatch(Array(arguments.dropFirst()))
    default:
        usage()
    }
} catch {
    FileHandle.standardError.write("error: \(error)\n".data(using: .utf8)!)
    exit(1)
}
