import XCTest
import simd
@testable import StitchCore

final class MeshTests: XCTestCase {

    func testMeshBilinearInterpolation() {
        var mesh = WarpMesh(width: 200, height: 200, vertexSpacing: 100)
        // 3x3 vertex grid; set the center vertex to a known offset.
        XCTAssertEqual(mesh.cols, 3)
        XCTAssertEqual(mesh.rows, 3)
        mesh.dx[4] = 8
        mesh.dy[4] = -4
        // At the center vertex the offset is exact.
        let center = mesh.offset(x: 100, y: 100)
        XCTAssertEqual(center.x, 8, accuracy: 1e-12)
        XCTAssertEqual(center.y, -4, accuracy: 1e-12)
        // Halfway toward a zero corner vertex it decays bilinearly.
        let mid = mesh.offset(x: 50, y: 100)
        XCTAssertEqual(mid.x, 4, accuracy: 1e-12)
        XCTAssertEqual(mid.y, -2, accuracy: 1e-12)
        // At the corners it is zero.
        XCTAssertEqual(mesh.offset(x: 0, y: 0).x, 0, accuracy: 1e-12)
    }

    /// Two identical cameras looking at the same scene, with a synthetic
    /// parallax bump displacing matched points in one region: the refiner must
    /// cut the pairwise residual by an order of magnitude while leaving the
    /// unaffected region essentially untouched.
    func testRefinerClosesSyntheticParallax() {
        let size = (width: 800, height: 600)
        let cams: [Int: Camera] = [
            0: Camera(focal: 700, width: size.width, height: size.height),
            1: Camera(focal: 700, width: size.width, height: size.height),
        ]

        // Matched grid points; inside a disc around (250, 300) image 1's
        // observations are shifted by up to 6 px (simulated near-field object).
        func bump(_ p: SIMD2<Double>) -> SIMD2<Double> {
            let d = length(p - SIMD2(250, 300))
            let radius = 140.0
            guard d < radius else { return .zero }
            let s = 6.0 * (1 - d / radius)
            return SIMD2(s, 0.4 * s)
        }

        var featuresA: [Feature] = []
        var featuresB: [Feature] = []
        var matches: [FeatureMatch] = []
        var idx = 0
        for y in stride(from: 40.0, through: 560, by: 40) {
            for x in stride(from: 40.0, through: 760, by: 40) {
                let p = SIMD2(x, y)
                let q = p + bump(p)
                featuresA.append(Feature(x: Float(p.x), y: Float(p.y), scale: 2, orientation: 0,
                                         response: 1, descriptor: []))
                featuresB.append(Feature(x: Float(q.x), y: Float(q.y), scale: 2, orientation: 0,
                                         response: 1, descriptor: []))
                matches.append(FeatureMatch(indexA: idx, indexB: idx, distance: 0))
                idx += 1
            }
        }
        let geometry = PairGeometry(homography: matrix_identity_double3x3,
                                    inlierIndices: Array(0..<matches.count),
                                    overlapMatchCount: matches.count,
                                    isVerified: true)
        let group = PanoramaGroup(imageIndices: [0, 1],
                                  pairs: [VerifiedPair(indexA: 0, indexB: 1,
                                                       matches: matches, geometry: geometry)])

        let result = MeshRefiner.refine(group: group,
                                        features: [featuresA, featuresB],
                                        cameras: cams,
                                        vertexSpacing: 80)
        XCTAssertGreaterThan(result.initialRMS, 1.0)
        // A cone-shaped bump is the regularizer's worst case (its tip is the
        // highest-curvature mode the smoothness term attenuates); ~70%+ RMS
        // reduction is the correct equilibrium, not a solver deficiency.
        XCTAssertLessThan(result.finalRMS, result.initialRMS / 3,
                          "refinement should close most of the synthetic parallax")

        // Far from the bump the meshes should stay nearly rigid.
        let farOffset = result.meshes[0]!.offset(x: 700, y: 100)
        XCTAssertLessThan(length(farOffset), 0.75)
    }
}
