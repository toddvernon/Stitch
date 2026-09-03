# Stitch

Automatic panorama stitching for macOS, in pure Swift. A modern replacement for
AutoStitch (Brown & Lowe), extended with parallax-tolerant alignment and
graph-cut seam finding. See [DESIGN.md](DESIGN.md) for the full pipeline and
roadmap; the source papers are in `docs/`.

Requires macOS 14+. No dependencies.

```sh
swift build -c release

# Milestone 1: SIFT feature detection with a debug overlay
.build/release/stitch features photo.jpg --debug-out keypoints.png
```

Status: milestone 1 (SIFT feature extraction) in progress.
