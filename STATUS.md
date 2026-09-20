# STATUS

Rolled at end of session, read by /sos. Current as of 2026-09-20.

## The session, in one breath

The repo went public (MIT LICENSE, photos untracked), the whole codebase got
a proper comment pass, and then I went after the sky band in the Hilton Head
panorama. Block-based gain compensation is built and behind a flag, but it
turned out not to be the whole answer: the residual across that seam is a
white-balance shift, not a brightness one.

## What's working

- Everything from before: rotational pipeline, strip mode, auto mode, GPS
  hints. Hilton Head still renders byte-identical to the reference with the
  default settings.
- Public repo at https://github.com/toddvernon/Stitch. Images/ is gitignored
  entirely; the house photos remain in earlier history, which I decided was
  fine.
- Comment pass (e68d156): file headers, doc comments on the public surface,
  paper citations at the algorithms, rationale at every magic number.
- Block gain (this session's last commit): `GainCompensator.solveBlocks`
  solves a coarse grid of gains per image (about 10 blocks across, tied by a
  smoothness term, same data and prior terms as the paper), `GainMap`
  interpolates it bilinearly, and the compositor applies it at both
  resolutions. `stitch pano --gain block` turns it on; the default is still
  `single`. The stitch log now prints each image's gain mean and range.
- 42 tests green.

## The sky band, what I learned

The visible tell in Hilton Head is the sky above the left trees, at the
seam between IMG_4001 and IMG_4002 (about 31% across). Things I ruled out
with measurements, so nobody has to redo them:

- Not the seam position. The graph cut puts it mid-overlap, not on a
  coverage edge. Seam locality 0.01 and 0.05 don't move it.
- Not blend depth. 8 pyramid levels instead of 5 leaves the sky profile
  identical to within a gray level.
- Only partly gain. Block gains close roughly half the step; weakening the
  prior (sigma_g 0.3 or 1.0) and the smoothness helps a little more, but
  the gain-corrected low-res mosaic still shows a clear step.
- The rest is color. Across the seam the sky differs by about 18% in red,
  12% in green, 8% in blue. The phone changed white balance and tone curve
  between frames (shutter went from 1/4673 to 1/7353 across the set). A
  luminance gain leaves a 10% red mismatch, which reads as the lighter,
  less saturated band.

So the fix is per-channel block gains: solve the same block system three
times, once per channel, with a weaker prior. Half a day. After that, if a
residual remains, it is tone-curve nonlinearity and the answer is shooting
with AE/AF lock.

## Known limits, not bugs

- IMG_4026 in BeachWalk has no overlap with anything (GPS says so).
- Near sand in a strip can never align; seams there are soft but findable.
- DESIGN.md says blend sigma 5 px; the blender uses sigma 2 per level. The
  code comment documents what the code does.
- `inlierCount` in PanoramaAligner.align is computed and never read.

## Mid-flight, needs a decision

- Package.swift has an uncommitted edit that did not come from this
  session (Dropbox-synced, dated Sep 18): it adds `-O` to StitchCore in
  Debug builds because Covey links this package by path. I left it
  unstaged rather than commit someone else's change blind. Commit it or
  revert it next session.
- Images/ has a dozen untracked sets from the Italy trip (Amalfi, Rome,
  Split, Sunrise). They stay local by design now.

## What's next

1. Per-channel block gains, then re-evaluate the Hilton Head sky; if it
   holds up, make block the default.
2. The open items from before: radial distortion in bundle adjustment,
   blender memory streaming, golden-image CI tests, app icon.
3. Three em-dashes remain in user-facing strings (CLI, app welcome text,
   GPS log line).

## Committed this session

edf6a3e and 18c8b84 public prep, e68d156 comment pass, then the block gain
commit and this roll. All on origin/main, verified in sync at /eos with 42
tests green.
