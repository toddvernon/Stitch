import Foundation

/// The compositor's view of an output space: a pixel grid plus a way to
/// project each source image into it. Gain compensation, seam finding,
/// blending, and crop all work on `ImageLayer`s and never see how the layers
/// were mapped, so the rotational panorama (`PanoLayerSource`) and the planar
/// strip (`StripGeometry`) share the whole compositing pass.
public protocol LayerSource {
    var width: Int { get }
    var height: Int { get }
    /// Images to composite, in the order they are added.
    var imageIndices: [Int] { get }
    /// Same mapping at a different output resolution.
    func scaled(toWidth newWidth: Int) -> Self
    /// Long-side source resolution that renders `imageIndex` at this
    /// output's resolution without upsampling (with sampling margin).
    func sourceDimension(for imageIndex: Int) -> Int
    func project(imageIndex: Int, image: RGBImage) -> ImageLayer?
}

/// The rotational panorama as a layer source: cameras, optional warp meshes,
/// and a `PanoGeometry`.
public struct PanoLayerSource: LayerSource {
    public var geometry: PanoGeometry
    public var cameras: [Int: Camera]
    public var meshes: [Int: WarpMesh]

    public init?(cameras: [Int: Camera], meshes: [Int: WarpMesh] = [:],
                 outputWidth: Int, projection: PanoProjection) {
        guard let g = PanoGeometry(cameras: cameras, outputWidth: outputWidth,
                                   projection: projection) else { return nil }
        geometry = g
        self.cameras = cameras
        self.meshes = meshes
    }

    private init(geometry: PanoGeometry, cameras: [Int: Camera], meshes: [Int: WarpMesh]) {
        self.geometry = geometry
        self.cameras = cameras
        self.meshes = meshes
    }

    public var width: Int { geometry.width }
    public var height: Int { geometry.height }
    public var imageIndices: [Int] { cameras.keys.sorted() }

    public func scaled(toWidth newWidth: Int) -> PanoLayerSource {
        PanoLayerSource(geometry: geometry.scaled(toWidth: newWidth), cameras: cameras, meshes: meshes)
    }

    /// Pano px/rad divided by the camera's px/rad, with sampling margin.
    public func sourceDimension(for imageIndex: Int) -> Int {
        let cam = cameras[imageIndex]!
        let needed = Double(max(cam.width, cam.height)) * geometry.scale / cam.focal * 1.2
        return Int(needed.rounded(.up))
    }

    public func project(imageIndex: Int, image: RGBImage) -> ImageLayer? {
        LayerProjector.project(imageIndex: imageIndex, camera: cameras[imageIndex]!,
                               image: image, mesh: meshes[imageIndex], geometry: geometry)
    }
}
