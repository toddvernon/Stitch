import XCTest
import simd
@testable import StitchCore

final class ProjectionTests: XCTestCase {

    func testForwardInverseRoundTrip() {
        for projection in PanoProjection.allCases {
            for theta in stride(from: -1.2, through: 1.2, by: 0.3) {
                for phi in stride(from: -0.7, through: 0.7, by: 0.35) {
                    let uv = projection.forward(theta: theta, phi: phi)
                    let back = projection.inverse(u: uv.x, v: uv.y)
                    XCTAssertEqual(back.theta, theta, accuracy: 1e-9,
                                   "\(projection) θ round trip at (\(theta), \(phi))")
                    XCTAssertEqual(back.phi, phi, accuracy: 1e-9,
                                   "\(projection) φ round trip at (\(theta), \(phi))")
                }
            }
        }
    }

    /// Near the pano center every projection must approximate u = θ, v = φ,
    /// so natural-width sizing keeps center resolution equal to the source.
    func testUnitScaleAtCenter() {
        for projection in PanoProjection.allCases {
            let uv = projection.forward(theta: 0.01, phi: 0.01)
            XCTAssertEqual(uv.x, 0.01, accuracy: 1e-4, "\(projection)")
            XCTAssertEqual(uv.y, 0.01, accuracy: 1e-4, "\(projection)")
        }
    }

    /// Pannini's defining property (d = 1): vertical scene lines stay
    /// straight — constant θ maps to constant u regardless of φ.
    func testPanniniKeepsVerticalsStraight() {
        for theta in [-1.0, -0.4, 0.5, 1.1] {
            let u0 = PanoProjection.pannini.forward(theta: theta, phi: -0.6).x
            let u1 = PanoProjection.pannini.forward(theta: theta, phi: 0.0).x
            let u2 = PanoProjection.pannini.forward(theta: theta, phi: 0.6).x
            XCTAssertEqual(u0, u1, accuracy: 1e-12)
            XCTAssertEqual(u1, u2, accuracy: 1e-12)
        }
    }

    func testGeometryPixelRoundTrip() {
        // A small camera rig; every pano pixel's direction must map back to
        // the same pixel under panoPoint, for each projection.
        let cams: [Int: Camera] = [
            0: Camera(rotation: SO3.exp(SIMD3(0, -0.4, 0)).transpose, focal: 700, width: 800, height: 600),
            1: Camera(rotation: SO3.exp(SIMD3(0, 0.4, 0)).transpose, focal: 700, width: 800, height: 600),
        ]
        for projection in PanoProjection.allCases {
            guard let geo = PanoGeometry(cameras: cams, outputWidth: 500, projection: projection) else {
                return XCTFail("geometry init failed for \(projection)")
            }
            for py in stride(from: 10, to: geo.height - 10, by: 40) {
                for px in stride(from: 10, to: geo.width - 10, by: 40) {
                    let d = geo.direction(px: Double(px), py: Double(py))
                    let p = geo.panoPoint(direction: d)
                    XCTAssertEqual(p.x, Double(px), accuracy: 1e-6, "\(projection)")
                    XCTAssertEqual(p.y, Double(py), accuracy: 1e-6, "\(projection)")
                }
            }
        }
    }

    func testAutoProjectionRule() {
        // ~53° span (two cameras ±0.4 rad, hfov ~60°): auto picks Pannini.
        let narrow: [Int: Camera] = [
            0: Camera(rotation: SO3.exp(SIMD3(0, -0.4, 0)).transpose, focal: 700, width: 800, height: 600),
            1: Camera(rotation: SO3.exp(SIMD3(0, 0.4, 0)).transpose, focal: 700, width: 800, height: 600),
        ]
        XCTAssertEqual(Compositor.resolveProjection(nil, cameras: narrow), .pannini)
        // ~200°+ span: auto falls back to spherical.
        let wide: [Int: Camera] = [
            0: Camera(rotation: SO3.exp(SIMD3(0, -1.5, 0)).transpose, focal: 700, width: 800, height: 600),
            1: Camera(rotation: SO3.exp(SIMD3(0, 0, 0)).transpose, focal: 700, width: 800, height: 600),
            2: Camera(rotation: SO3.exp(SIMD3(0, 1.5, 0)).transpose, focal: 700, width: 800, height: 600),
        ]
        XCTAssertEqual(Compositor.resolveProjection(nil, cameras: wide), .spherical)
        // An explicit choice always wins.
        XCTAssertEqual(Compositor.resolveProjection(.cylindrical, cameras: narrow), .cylindrical)
    }
}
