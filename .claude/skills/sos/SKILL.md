---
name: sos
description: Start of session. Run first thing when starting or resuming Stitch work (especially after moving between Macs). Syncs the repo from origin, surfaces divergence or leftover uncommitted work instead of plowing ahead, reorients from STATUS.md, and confirms the build and tests are green before anything is built on top of them.
---

# /sos -- start of session

Goal: never build on stale or broken state, and resume with context. Run all
steps in order, then report a short summary. Do NOT start other work until this
finishes or until the user resolves anything flagged.

## 1. Sync the repo

`git fetch -q`, then classify against origin/main and act:

- **Up to date** -> note OK.
- **Behind, clean working tree** -> `git pull --ff-only` (safe; the common
  "other Mac pushed" case).
- **Diverged** (local commits the remote doesn't have) -> **STOP. Do not merge
  or pull.** Report it -- a prior session probably didn't `/eos`, or both Macs
  worked. Wait for the user.
- **Dirty working tree** -> report "uncommitted work was left here last time";
  do not pull over it. Wait for the user.

Caveat: this repo rides Dropbox (reachable as both
`~/Library/CloudStorage/Dropbox/dev/Stitch` and `~/Dropbox/dev/Stitch`). If
the files look newer than the git state -- Dropbox synced edits the local
history doesn't know -- flag it for the user rather than guessing. Dropbox
also means `.build/` may have been synced from the other Mac; if the build
behaves strangely, `rm -rf .build` and rebuild before suspecting the code.

Untracked photo sets under `Images/` (folders of source shots and their
stitched `<folder>.jpg`) are expected and are left untracked on purpose
until the user decides to commit them; don't flag them as leftover work.
`Images/` is gitignored entirely; nothing under it is committed (the repo is public).

## 2. Reorient

Read `STATUS.md` and give a 3-line summary: where we left off and the top
"what's next" items.

## 3. Build and test glance

This project has no live installation; the repo is the whole state, so the
check is that it builds and its tests pass:

- `swift build -c release 2>&1 | tail -3` -- must end in `Build complete`.
- `swift test 2>&1 | grep -E "Executed|error:|failed" | tail -3` -- the
  final `Executed N tests, with 0 failures` line is what matters.

A failure here on a clean, up-to-date tree is worth stopping for (toolchain
change, Dropbox-synced `.build`, or a commit that went out red).

## 4. Report

One short block: repo state (pulled / flagged), the resume context from
STATUS.md, build and test result. Then we're ready to work.
