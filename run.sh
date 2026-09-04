#!/bin/zsh
# Build (incrementally) and run Stitch.
#
#   ./run.sh                  open the app
#   ./run.sh <folder>         stitch that folder from the command line;
#                             output lands next to it as <folder>.jpg
#   ./run.sh <stitch args>    pass anything else straight to the stitch CLI,
#                             e.g. ./run.sh pano shots -o pano.png --no-crop
set -e
cd "$(dirname "$0")"

if [[ $# -eq 0 ]]; then
    Scripts/make-app.sh
    open Stitch.app
elif [[ -d $1 && $# -eq 1 ]]; then
    swift build -c release
    folder="${1%/}"
    out="${folder:A}.jpg"
    .build/release/stitch pano "$folder" -o "$out"
else
    swift build -c release
    .build/release/stitch "$@"
fi
