#!/bin/zsh
# Builds Stitch.app from the SwiftPM StitchApp executable.
set -e
cd "$(dirname "$0")/.."

swift build -c release --product StitchApp

APP=Stitch.app
rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS"

cat > "$APP/Contents/Info.plist" <<'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleName</key>              <string>Stitch</string>
    <key>CFBundleDisplayName</key>       <string>Stitch</string>
    <key>CFBundleIdentifier</key>        <string>com.toddvernon.stitch</string>
    <key>CFBundleExecutable</key>        <string>StitchApp</string>
    <key>CFBundleVersion</key>           <string>1.0</string>
    <key>CFBundleShortVersionString</key><string>1.0</string>
    <key>CFBundlePackageType</key>       <string>APPL</string>
    <key>LSMinimumSystemVersion</key>    <string>14.0</string>
    <key>NSHighResolutionCapable</key>   <true/>
</dict>
</plist>
EOF

cp .build/release/StitchApp "$APP/Contents/MacOS/StitchApp"
codesign --force --sign - "$APP" 2>/dev/null || true
echo "built $APP — open it with: open $APP"
