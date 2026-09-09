import Foundation
import simd

// The per-image warp of the parallax stage (DESIGN.md stage 7). MeshRefiner
// solves the vertex offsets; LayerProjector (PanoGeometry.swift) applies
// them when it maps pano pixels back into each source image, and pads each
// layer's bounding box by `maxOffset`.

/// A coarse grid of 2D offsets over an image, bilinearly interpolated.
/// `offset(x:y:)` gives d(p) where the corrected position of image content at
/// pixel p is W(p) = p + d(p). Offsets are at registration scale.
public struct WarpMesh {
    /// Vertex grid size; the outer vertices sit exactly on the image edges.
    public let cols: Int
    public let rows: Int
    /// Image size the mesh spans, px at registration scale.
    public let width: Int
    public let height: Int
    /// Vertex offsets, row-major rows x cols.
    public var dx: [Double]
    public var dy: [Double]

    /// An all-zero (identity) mesh with vertices at most `vertexSpacing` px
    /// apart: the count is rounded up, so the actual spacing shrinks to fit
    /// the image exactly. Always at least a 2x2 grid.
    public init(width: Int, height: Int, vertexSpacing: Int) {
        self.width = width
        self.height = height
        self.cols = max(2, Int((Double(width) / Double(vertexSpacing)).rounded(.up)) + 1)
        self.rows = max(2, Int((Double(height) / Double(vertexSpacing)).rounded(.up)) + 1)
        dx = [Double](repeating: 0, count: cols * rows)
        dy = [Double](repeating: 0, count: cols * rows)
    }

    /// Bilinear weights of the four vertices around pixel (x, y):
    /// (vertexIndex, weight) with weights summing to 1.
    public func stencil(x: Double, y: Double) -> [(index: Int, weight: Double)] {
        // Clamp to the image so points a hair outside it (sub-pixel feature
        // positions near the border) still fall in the edge cell.
        let gx = min(max(x, 0), Double(width)) / Double(width) * Double(cols - 1)
        let gy = min(max(y, 0), Double(height)) / Double(height) * Double(rows - 1)
        let c0 = min(Int(gx), cols - 2)
        let r0 = min(Int(gy), rows - 2)
        let fx = gx - Double(c0)
        let fy = gy - Double(r0)
        return [
            (r0 * cols + c0, (1 - fx) * (1 - fy)),
            (r0 * cols + c0 + 1, fx * (1 - fy)),
            ((r0 + 1) * cols + c0, (1 - fx) * fy),
            ((r0 + 1) * cols + c0 + 1, fx * fy),
        ]
    }

    /// Interpolated offset d(p) at pixel p.
    public func offset(x: Double, y: Double) -> SIMD2<Double> {
        var ox = 0.0, oy = 0.0
        for (i, w) in stencil(x: x, y: y) {
            ox += w * dx[i]
            oy += w * dy[i]
        }
        return SIMD2(ox, oy)
    }

    /// Largest vertex displacement, px. Bounds how far any content moved,
    /// which is what the renderer needs to pad a layer's bounding box.
    public var maxOffset: Double {
        var m = 0.0
        for i in 0..<(cols * rows) {
            m = max(m, sqrt(dx[i] * dx[i] + dy[i] * dy[i]))
        }
        return m
    }
}
