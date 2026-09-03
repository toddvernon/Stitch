# Stitch

Automatic panorama stitching for macOS, in pure Swift. A modern replacement for
AutoStitch (Brown & Lowe), extended with parallax-tolerant alignment and
graph-cut seam finding. See [DESIGN.md](DESIGN.md) for the full pipeline and
roadmap; the source papers are in `docs/`.

Requires macOS 14+. No dependencies.

```sh
swift build -c release

# Stitch a folder of photos into a panorama (linear-blend preview quality)
.build/release/stitch pano Images/HiltonHeadHouse -o pano.png

# Inspect the pipeline stage by stage
.build/release/stitch features photo.jpg --debug-out keypoints.png
.build/release/stitch match a.jpg b.jpg --debug-out matches.png
.build/release/stitch recognize Images/HiltonHeadHouse
```

Status: milestones 1-4 done (SIFT, matching/RANSAC/verification, panorama
recognition, bundle adjustment + straightening + linear-blend render). Next:
mesh-based parallax refinement, then gain compensation, graph-cut seams, and
multi-band blending.
