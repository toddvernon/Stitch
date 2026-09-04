---
name: eos
description: End of session. Run when wrapping up Stitch work so the next session (here or on the other Mac) starts clean. Runs the tests, rolls STATUS.md, commits and pushes to main, verifies sync with origin, and reminds about anything worth remembering.
---

# /eos -- end of session

Goal: make this a clean handoff so nothing is lost and the next session can
pick up cold. Run all steps, then report a "safe to walk away" summary the
user can eyeball.

## 1. Tests

`swift test 2>&1 | grep -E "Executed|error:|failed" | tail -3`. Never push
red: if something fails, fix it or tell the user before going further. If a
stitching change was made, a real-image run is part of "tested" -- e.g.
`./run.sh Images/HiltonHeadHouse` for the rotational pipeline or
`./run.sh Images/BeachWalk` for strips -- and its output was looked at.

## 2. Roll STATUS.md

Overwrite `STATUS.md` with the current state: what's working, what's broken,
what's next, what's committed (with short hashes), and anything mid-flight
(a tuning value under evaluation, a photo set that needs reshooting).
Casual first-person voice, no em-dashes. **Never** create dated
`STATUS_YYYY-MM-DD.md` files -- git log preserves history.

## 3. Commit + push

Run `git status`. If there are changes:

- **Stage selectively** -- intended source/docs/tests only. Do NOT blanket
  `git add .`. Photo sets under `Images/` and their stitched outputs are
  tens of MB each; never stage them without asking (the untracked ones are
  untracked on purpose). Skip `Stitch.app/`, `.build/`, scratch output, and
  anything generated. If an untracked file's intent is unclear, ask before
  adding it.
- Commit straight to **main** (never branch -- the two-Mac sync runs through
  origin/main), in this repo's message style: `Area: what changed, in plain
  words` (see git log -- e.g. `Strip mode: ...`, `Help output: ...`). End
  with the session's standard Claude co-author trailer (whatever the current
  harness specifies -- don't hardcode a model name here, it goes stale).
- `git push`. If the push is **rejected** (divergence), STOP and walk the
  merge with the user -- never force-push. Surface what's on the remote that
  isn't local.

## 4. Verify sync

Confirm the repo is ahead 0 / behind 0 vs. origin/main. Report the pushed
short hashes (this is the summary the user looks at to confirm the session
was closed out).

## 5. Memory

Save anything worth persisting to memory (decisions, tuning values and why,
gotchas that cost time this session).

## 6. Report

A short "safe to walk away" block: pushed hashes, STATUS rolled, tests
green, and anything left deliberately uncommitted.
