import Foundation

/// A planar single-channel float image, row-major, values nominally in [0, 1].
/// This is the working currency of the registration pipeline.
public struct ImageF {
    public let width: Int
    public let height: Int
    public var pixels: [Float]

    public init(width: Int, height: Int, pixels: [Float]) {
        precondition(pixels.count == width * height)
        self.width = width
        self.height = height
        self.pixels = pixels
    }

    public init(width: Int, height: Int, fill: Float = 0) {
        self.width = width
        self.height = height
        self.pixels = [Float](repeating: fill, count: width * height)
    }

    @inlinable
    public subscript(x: Int, y: Int) -> Float {
        get { pixels[y * width + x] }
        set { pixels[y * width + x] = newValue }
    }

    /// Bilinear sample; clamps to the image border.
    public func sample(x: Float, y: Float) -> Float {
        let cx = min(max(x, 0), Float(width - 1))
        let cy = min(max(y, 0), Float(height - 1))
        let x0 = min(Int(cx), width - 2 < 0 ? 0 : width - 2)
        let y0 = min(Int(cy), height - 2 < 0 ? 0 : height - 2)
        let fx = cx - Float(x0)
        let fy = cy - Float(y0)
        let x1 = min(x0 + 1, width - 1)
        let y1 = min(y0 + 1, height - 1)
        let top = self[x0, y0] * (1 - fx) + self[x1, y0] * fx
        let bot = self[x0, y1] * (1 - fx) + self[x1, y1] * fx
        return top * (1 - fy) + bot * fy
    }
}
