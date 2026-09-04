# Stitch — Design

A personal macOS panorama stitcher in Swift, replacing AutoStitch (Matthew Brown's
application, no longer supported on macOS). The reference algorithm is Brown & Lowe,
"Automatic Panoramic Image Stitching using Invariant Features" (IJCV 2007), extended
with the improvements the field converged on afterward: parallax-tolerant local
alignment, graph-cut seam finding, radial distortion in the camera model, and
block-based gain compensation. Both source papers are in `docs/`.

## Goals

1. Fully automatic: point it at a folder of images, get panoramas. No ordering,
   no orientation hints, no manual control points. Junk images are ignored,
   multiple panoramas in one folder are recognized separately.
2. Parallax tolerant. The single biggest failure of AutoStitch in practice is
   handheld rotation about the photographer rather than the lens's no-parallax
   point: near objects (a railing you are standing next to) ghost and break.
   Stitch treats this as a core requirement, not an edge case.
3. Native and dependency-free: pure Swift + Apple frameworks (Accelerate, Image I/O,
   Core Graphics). No OpenCV, no bundled neural networks, no third-party packages
   in the core. Apple platforms only (macOS 14+), and unapologetically so.
4. Open-source shaped from day one: MIT-licensable code, library/CLI/app separation,
   every pipeline stage testable in isolation.

## Non-goals

- Cross-platform support (Linux/Windows).
- Deep-learning or generative stitching. Learned matchers may slot in later as an
  optional module (see Architecture), but the core is classical geometry, and no
  pixel is ever synthesized.
- Video, real-time preview during capture, gigapixel tiling (revisit later if wanted).

## Pipeline

Stages 1–6 are Brown & Lowe (IJCV 2007) with their published constants; stages
7–10 are the post-2007 upgrades. Input images are optionally downsampled for
feature detection and registration; rendering always uses full resolution.

1. **SIFT feature extraction** on every image. Scale/rotation invariant keypoints
   with 128-dim descriptors. Standard parameters: 3 scales/octave, σ₀ = 1.6,
   contrast threshold 0.04 (OpenCV convention), edge threshold r = 10.
   (SIFT's patent expired in 2020.)
2. **Matching**: each feature matched to its k = 4 nearest neighbors across all
   other images via a k-d tree.
3. **Pairwise verification**: for each image take the m = 6 images with most raw
   matches; RANSAC (r = 4 correspondences, DLT homography, 500 trials); accept
   the pair as a true overlap if inliers nᵢ > α + β·n_f with α = 8.0, β = 0.3,
   where n_f is the feature count in the overlap region.
4. **Panorama recognition**: connected components of verified pairs. Components
   of size 1 are discarded.
5. **Global bundle adjustment**: each camera is a rotation θ ∈ so(3) plus focal
   length f, **plus one or two radial distortion coefficients** (upgrade over the
   paper, which flags this in its Future Work; their Figure 9 shows the artifacts).
   EXIF focal length initializes f when present. Levenberg–Marquardt over
   robustified reprojection error (Huber, σ = 2 px final), images added best-first,
   analytic Jacobians, sparse J^T J accumulation as in §4.1 of the paper.
6. **Automatic straightening**: up-vector from the null eigenvector of the
   covariance of camera X-axes (paper §5).
7. **Local mesh refinement** (parallax stage; Zhang & Liu CVPR 2014 / APAP family).
   A coarse vertex mesh (~40×30) per image, solved as one sparse linear
   least-squares problem: data term pulls surviving feature matches into
   registration; regularization keeps each cell near a similarity transform so
   straight lines stay straight. Far content barely moves; near content (the
   railing) deforms locally to close the parallax gap. Skipped automatically when
   Stage-5 residuals are already small (tripod shots stay perfectly rigid).
8. **Gain compensation, block-based**: closed-form least-squares gains solved on a
   grid of blocks and bilinearly interpolated (handles vignetting and sky
   gradients; upgrade over the paper's single gain per image, σ_N = 10, σ_g = 0.1).
9. **Graph-cut seam finding** (Kwatra 2003 / Agarwala 2004): optimal seams through
   overlap regions route around moving objects and residual parallax, so each
   object comes from exactly one photo. Max-flow solver implemented in-house
   (the well-known BK maxflow *code* is research-only licensed; the algorithm is
   free — we write our own).
10. **Multi-band blending** (Burt–Adelson, 5 bands, σ = 5 px) across the seams,
    rendered in spherical coordinates (θ, φ). Output: TIFF/JPEG, plus the option
    of equirectangular output for 360° sets.

## Why both mesh warps and seams

They fail differently. Seams alone hide ghosting but leave near objects subtly
bent at each cut; mesh warps alone leave faint double edges where residuals
remain. Mesh warping closes most of the parallax gap, then the seam finder
resolves what's left by choosing one source image per region. Extreme parallax
(inches from the lens, large stance shifts) remains out of scope — the photos
genuinely contain different occlusions and no warp can reconcile them.

## Architecture

One SwiftPM package, three products:

```
Stitch/
├── Package.swift
├── Sources/
│   ├── StitchCore/          # the library; no UI, no I/O policy, every stage testable
│   │   ├── Image/           # planar float images, Image I/O loading, EXIF, debug rendering
│   │   ├── Features/        # SIFT (FeatureExtractor protocol + SIFTDetector)
│   │   ├── Matching/        # k-d tree, RANSAC, pair verification, panorama recognition
│   │   ├── Geometry/        # camera model, bundle adjustment, straightening, mesh warp
│   │   └── Compositing/     # gain, seams (max-flow), multi-band blend, spherical render
│   └── StitchCLI/           # `stitch` command-line tool; drives the pipeline, test harness
├── Tests/StitchCoreTests/
├── docs/                    # Brown & Lowe papers, this file’s references
└── (later) StitchApp/       # thin SwiftUI app via Xcode project
```

Key seams (the software kind):

- `FeatureExtractor` protocol with `SIFTDetector` as the sole built-in
  implementation. A Core ML SuperPoint/LightGlue extractor can be added later as
  a separate optional module without touching the core (also keeps
  non-commercially-licensed weights out of the MIT core).
- Registration (stages 1–7) and compositing (8–10) communicate through one value
  type: per-image camera parameters + optional mesh. The renderer doesn't know or
  care how alignment was obtained, so alternative warps slot in cleanly.
- Every stage takes and returns plain value types and is callable from the CLI
  for inspection (`stitch features`, `stitch match`, `stitch align`, …).

## Performance

Accelerate throughout: vImage for pyramids and convolution, vDSP for vector math,
LAPACK (via Accelerate) for the bundle-adjustment and mesh linear solves.
Registration runs on downsampled images (configurable cap, ~2–3 MP), like
AutoStitch did. Rendering at full resolution is the expensive step; start with
CPU + vImage, move pyramid construction and resampling to Metal compute if it
matters. The 2007 paper did 57 images on a 1.6 GHz Pentium M; an Apple Silicon
Mac has headroom to spare.

## Testing

- Unit tests per stage (synthetic images with known ground truth: blobs at known
  scales, known homographies, known gains).
- Golden-image tests: checked-in small shot sets — including a handheld railing
  set as test case zero — stitched in CI and diffed perceptually against blessed
  outputs.

## Licensing (for open-sourcing)

MIT. Constraints already designed in: SIFT patent expired; own max-flow
implementation; no GPL/research-only code; learned-feature weights only ever in
an optional non-core module.

## Milestones

1. **SIFT**: extraction with CLI debug rendering of keypoints. ✓
2. **Matching + RANSAC + verification** on two overlapping shots. ✓
3. **Panorama recognition** on a messy folder. ✓
4. **Bundle adjustment + straightening**, crude linear-blend render to verify alignment. ✓
5. **Mesh refinement** (the railing fix). ✓
6. **Gain + graph-cut seams + multi-band blending** — full-quality output. ✓
7. **SwiftUI app**. ✓

All milestones complete. Open items: radial distortion in bundle adjustment,
memory streaming in the blender, golden-image CI tests, app icon.

## References

- M. Brown, D. Lowe. *Automatic Panoramic Image Stitching using Invariant
  Features.* IJCV 2007. (`docs/brown-lowe-ijcv2007.pdf`)
- M. Brown, D. Lowe. *Recognising Panoramas.* ICCV 2003. (`docs/brown-lowe-iccv2003.pdf`)
- D. Lowe. *Distinctive Image Features from Scale-Invariant Keypoints.* IJCV 2004.
- F. Zhang, F. Liu. *Parallax-tolerant Image Stitching.* CVPR 2014.
- J. Zaragoza et al. *As-Projective-As-Possible Image Stitching with Moving DLT.* CVPR 2013.
- V. Kwatra et al. *Graphcut Textures.* SIGGRAPH 2003.
- A. Agarwala et al. *Interactive Digital Photomontage.* SIGGRAPH 2004.
- P. Burt, E. Adelson. *A Multiresolution Spline with Application to Image Mosaics.* ToG 1983.
