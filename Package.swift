// swift-tools-version: 5.9
import PackageDescription

let package = Package(
    name: "Stitch",
    platforms: [.macOS(.v14)],
    products: [
        .library(name: "StitchCore", targets: ["StitchCore"]),
        .executable(name: "stitch", targets: ["StitchCLI"]),
        .executable(name: "StitchApp", targets: ["StitchApp"]),
    ],
    targets: [
        .target(name: "StitchCore"),
        .executableTarget(name: "StitchCLI", dependencies: ["StitchCore"]),
        .executableTarget(name: "StitchApp", dependencies: ["StitchCore"]),
        .testTarget(name: "StitchCoreTests", dependencies: ["StitchCore"]),
    ]
)
