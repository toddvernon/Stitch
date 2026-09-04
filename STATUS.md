# STATUS

Rolled at end of session, read by /sos. Current as of 2026-09-04 afternoon.

## The session, in one breath

Strip mode landed. The BeachWalk set (walking along the row of houses at
Hilton Head, one shot per house) now stitches into a single multi-viewpoint
strip, and the app and CLI pick panorama or strip on their own. Then GPS
hints went in as pure diagnostics, and the repo got its first remote:
https://github.com/toddvernon/Stitch, private for now.

## What's working

- The full rotational pipeline (milestones 1 through 7) unchanged, and
  verified unchanged: Hilton Head renders identically and in the same 25 s
  on the pre-strip build. The README's 14 s figure predates the Pannini
  projection and the wider natural width; it was stale before today.
- Strip mode (39e7d06). Similarity pair model with 2-point RANSAC, global
  linear solve with Huber IRLS, planar output through a LayerSource
  protocol the compositor is now generic over, and a viewpoint-locality
  term in the graph cut. Strips register at 3000 px (2000 only chained 3
  of the 5 connectable BeachWalk photos) and blend with 8 pyramid levels
  (5 left a vertical band in the sky). BeachWalk: 5 photos, 13020x3885,
  about 20 s. Seam locality is 0.01; 0 let a seam run through a house and
  duplicate it, 0.005 and 0.02 both looked fine, so I split the difference.
- Auto mode: both recognizers run on the same features and whichever
  places more images wins, panorama on a tie. Strip winning triggers a
  re-detect at 3000 px. run.sh on a folder therefore just does the right
  thing.
- GPS hints (d87d6da). Advisory only, never required, never in the
  geometry. Log line with span and typical step, an explanation for
  unplaced photos that sit far beyond the typical step, and the auto-mode
  tiebreak. Verified silent and identical on metadata-stripped copies.
- 39 tests green.

## Known limits, not bugs

- IMG_4026 in BeachWalk has no overlap with anything: 4024 and 4025 were
  dropped, and the GPS line now says so (60 m from its neighbor, 2.2x the
  typical step). Reshoot without gaps if that end of the row matters.
- Near sand in a strip can never align (different viewpoints, content off
  the facade plane); seams there are soft but findable. Expected per
  DESIGN.md.

## What's next

- Decide about the untracked photo sets: Images/BeachWalk, Images/Sunrise
  and their stitched outputs (about 130 MB) are sitting untracked. Commit
  them as test sets like HiltonHeadHouse, or ignore them.
- The open items from before, unchanged: radial distortion in bundle
  adjustment, blender memory streaming, golden-image CI tests, app icon.
- If the repo ever goes public: add the MIT LICENSE file DESIGN.md
  promises, and think about whether the house photos should ship with it.

## Committed this session

39e7d06 strip mode, d87d6da GPS hints, plus the /sos and /eos skills and
this file. All on origin/main.
