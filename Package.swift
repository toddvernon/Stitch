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
        // Optimized even in a host's Debug configuration: the pipeline is
        // numeric Swift that runs 10-50x slower at -Onone, which turns a
        // 25 s panorama into a quarter hour when Covey links this package
        // from its Debug build. Allowed because the package is referenced
        // by path, never as a remote dependency.
        .target(
            name: "StitchCore",
            swiftSettings: [.unsafeFlags(["-O"], .when(configuration: .debug))]
        ),
        .executableTarget(name: "StitchCLI", dependencies: ["StitchCore"]),
        .executableTarget(name: "StitchApp", dependencies: ["StitchCore"]),
        .testTarget(name: "StitchCoreTests", dependencies: ["StitchCore"]),
    ]
)
