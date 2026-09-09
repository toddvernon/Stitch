# Stitch

Automatic panorama stitching for macOS, in pure Swift. A modern replacement for
AutoStitch (Brown & Lowe), extended with parallax-tolerant alignment,
graph-cut seam finding, and a multi-viewpoint strip mode for photos taken
walking along a row of houses. See [DESIGN.md](DESIGN.md) for the full pipeline and
roadmap; the source papers are in `docs/`.

Requires macOS 14+. No dependencies.

```sh
# The app: drop a folder of photos, preview, export
Scripts/make-app.sh && open Stitch.app

# The CLI: stitch a folder into full-resolution panoramas
swift build -c release
.build/release/stitch pano Images/MyPhotos -o pano.jpg

# A walk along a facade: one long multi-viewpoint strip
# (pano's default --mode auto picks this by itself when it fits better)
.build/release/stitch strip Images/BeachWalk -o strip.jpg

# Inspect the pipeline stage by stage
.build/release/stitch features photo.jpg --debug-out keypoints.png
.build/release/stitch match a.jpg b.jpg --debug-out matches.png
.build/release/stitch recognize Images/MyPhotos
```

Status: all seven milestones done — the full stitching pipeline (SIFT,
matching/RANSAC/verification, panorama recognition, bundle adjustment +
straightening, mesh-based parallax refinement, gain compensation, graph-cut
seams, multi-band blending, auto-crop) at the sources' native resolution
(a 6-shot iPhone set renders a ~40 MP panorama in ~14 s), plus a drag-and-drop
SwiftUI app. Strip mode places walked-along shots on the facade plane with
similarity transforms solved globally and lets the seam finder do the
multi-viewpoint selection (Agarwala 2006). Open items: radial distortion in
bundle adjustment, blender memory streaming, golden-image CI tests, app icon.
