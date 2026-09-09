// Compositing stage, output-space abstraction. Registration hands the
// compositor a LayerSource; everything downstream (gain, seams, blend, crop)
// works on the ImageLayers it produces and never touches cameras or
// similarities directly.

import Foundation

/// The compositor's view of an output space: a pixel grid plus a way to
/// project each source image into it. Gain compensation, seam finding,
/// blending, and crop all work on `ImageLayer`s and never see how the layers
/// were mapped, so the rotational panorama (`PanoLayerSource`) and the planar
/// strip (`StripGeometry`) share the whole compositing pass.
public protocol LayerSource {
    /// Output grid size in pixels.
    var width: Int { get }
    var height: Int { get }
    /// Images to composite, in the order they are added.
    var imageIndices: [Int] { get }
    /// Same mapping at a different output resolution.
    func scaled(toWidth newWidth: Int) -> Self
    /// Long-side source resolution that renders `imageIndex` at this
    /// output's resolution without upsampling (with sampling margin).
    func sourceDimension(for imageIndex: Int) -> Int
    /// Resamples `image` into its bounding box in output space. The image may
    /// be at any resolution (it is mapped through its size ratio to the
    /// registration-scale geometry); nil if nothing lands inside the output.
    func project(imageIndex: Int, image: RGBImage) -> ImageLayer?
}

/// The rotational panorama as a layer source: cameras, optional warp meshes,
/// and a `PanoGeometry`.
public struct PanoLayerSource: LayerSource {
    /// Angular extent, projection, and the pixel-to-ray mapping.
    public var geometry: PanoGeometry
    /// Bundle-adjusted cameras at registration scale, keyed by image index.
    public var cameras: [Int: Camera]
    /// Parallax meshes from mesh refinement; images without one render rigidly.
    public var meshes: [Int: WarpMesh]

    /// Builds the geometry from the cameras' angular extent. nil when the
    /// cameras cover no finite extent (empty set, or degenerate rays).
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
        // geometry.scale is pano px per projection unit (about px/rad at the
        // center); focal is source px/rad. Their ratio is output px per source
        // px, and the 1.2 gives bilinear sampling a little headroom over 1:1.
        let needed = Double(max(cam.width, cam.height)) * geometry.scale / cam.focal * 1.2
        return Int(needed.rounded(.up))
    }

    public func project(imageIndex: Int, image: RGBImage) -> ImageLayer? {
        LayerProjector.project(imageIndex: imageIndex, camera: cameras[imageIndex]!,
                               image: image, mesh: meshes[imageIndex], geometry: geometry)
    }
}
