import XCTest
import simd
@testable import StitchCore

final class GeometryTests: XCTestCase {

    // MARK: - SO(3)

    func testRodriguesMatchesKnownRotation() {
        let angle = 0.7
        let r = SO3.exp(SIMD3(0, angle, 0))  // rotation about y
        let expected = simd_double3x3(rows: [
            SIMD3(cos(angle), 0, sin(angle)),
            SIMD3(0, 1, 0),
            SIMD3(-sin(angle), 0, cos(angle)),
        ])
        for c in 0..<3 {
            for rIdx in 0..<3 {
                XCTAssertEqual(r[c][rIdx], expected[c][rIdx], accuracy: 1e-12)
            }
        }
        XCTAssertEqual(SO3.exp(.zero).determinant, 1, accuracy: 1e-12)
    }

    func testOrthonormalizeRecoversRotation() {
        let r = SO3.exp(SIMD3(0.3, -0.5, 0.2))
        let perturbed = r * simd_double3x3(diagonal: SIMD3(1.05, 0.97, 1.01))
        let fixed = SO3.orthonormalize(perturbed)
        let identity = fixed * fixed.transpose
        for c in 0..<3 {
            for rIdx in 0..<3 {
                XCTAssertEqual(identity[c][rIdx], c == rIdx ? 1 : 0, accuracy: 1e-9)
            }
        }
        XCTAssertEqual(fixed.determinant, 1, accuracy: 1e-9)
    }

    // MARK: - Synthetic bundle adjustment

    /// Ground-truth camera ring, observations from projected world points,
    /// perturbed initialization; BA must recover geometry to sub-pixel RMS
    /// and the true relative rotation angles.
    func testBundleAdjustmentRecoversSyntheticRig() {
        let f = 800.0
        let size = (width: 1200, height: 900)
        let angles = [-0.5, 0.0, 0.5]
        let truth = angles.map { a in
            Camera(rotation: SO3.exp(SIMD3(0, a, 0)).transpose, focal: f,
                   width: size.width, height: size.height)
        }
        // Note: Camera.rotation is world→camera; a camera panned by +a about y
        // has R = exp([0,a,0])ᵀ.

        var rng = SplitMix64(seed: 1234)
        func uniform(_ lo: Double, _ hi: Double) -> Double {
            lo + (hi - lo) * (Double(rng.next() >> 11) / Double(1 << 53))
        }

        var observations: [MatchObservation] = []
        var pointCount = 0
        while pointCount < 400 {
            // Random world directions spanning the rig's field of view.
            let theta = uniform(-0.9, 0.9)
            let phi = uniform(-0.45, 0.45)
            let d = SIMD3(sin(theta) * cos(phi), -sin(phi), cos(theta) * cos(phi))
            var seen: [(Int, SIMD2<Double>)] = []
            for (i, cam) in truth.enumerated() {
                if let p = cam.project(d),
                   abs(p.x) < Double(size.width) / 2 - 10,
                   abs(p.y) < Double(size.height) / 2 - 10 {
                    seen.append((i, p))
                }
            }
            guard seen.count >= 2 else { continue }
            pointCount += 1
            for a in 0..<seen.count {
                for b in 0..<seen.count where a != b {
                    observations.append(MatchObservation(cameraA: seen[a].0, cameraB: seen[b].0,
                                                         pointA: seen[a].1, pointB: seen[b].1))
                }
            }
        }

        // Perturbed start: wrong focal, noisy rotations.
        var cameras = truth.enumerated().map { (i, cam) in
            Camera(rotation: cam.rotation * SO3.exp(SIMD3(uniform(-0.05, 0.05),
                                                          uniform(-0.05, 0.05),
                                                          uniform(-0.05, 0.05))),
                   focal: 650, width: cam.width, height: cam.height)
        }

        let rms = BundleAdjuster.adjust(cameras: &cameras, observations: observations,
                                        huberSigma: nil, maxIterations: 100)
        XCTAssertLessThan(rms, 0.01, "synthetic noise-free rig should converge to ~zero error")

        for cam in cameras {
            XCTAssertEqual(cam.focal, f, accuracy: f * 0.01)
        }
        // Relative rotation between outer cameras should be 1.0 rad (gauge-free check).
        let rel = cameras[0].rotation * cameras[2].rotation.transpose
        let angle = acos(max(-1, min(1, (rel[0][0] + rel[1][1] + rel[2][2] - 1) / 2)))
        XCTAssertEqual(angle, 1.0, accuracy: 0.01)
    }

    func testStraightenAlignsUpVector() {
        // Cameras panned about world y with a consistent small twist applied
        // to the whole rig: straightening must undo the twist.
        let twist = SO3.exp(SIMD3(0, 0, 0.3))  // roll
        var cameras: [Int: Camera] = [:]
        for (i, a) in [-0.6, -0.2, 0.2, 0.6].enumerated() {
            let r = SO3.exp(SIMD3(0, a, 0)).transpose * twist
            cameras[i] = Camera(rotation: r, focal: 800, width: 1200, height: 900)
        }
        PanoramaAligner.straighten(cameras: &cameras)
        // After straightening, every camera X axis should be horizontal
        // (zero y component) since the rig only pans.
        for cam in cameras.values {
            let xAxis = cam.rotation.transpose.columns.0
            XCTAssertEqual(xAxis.y, 0, accuracy: 1e-9)
            // For a pure-pan rig the camera up vector must coincide with the
            // world vertical after straightening (y points down, so up.y = -1).
            let up = -cam.rotation.transpose.columns.1
            XCTAssertEqual(up.y, -1, accuracy: 1e-9)
        }
    }
}
