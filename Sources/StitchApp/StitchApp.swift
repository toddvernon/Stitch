import AppKit
import SwiftUI

// App entry point: one window, and ContentView does everything in it.
// Scripts/make-app.sh wraps the SwiftPM binary in a bundle for normal use;
// the activation dance below is for running the bare binary during
// development, where macOS otherwise treats it as a background process.

/// The single-window SwiftUI app. All state lives in `StitchModel`, owned
/// by `ContentView`; the app struct only sets up the scene.
@main
struct StitchApp: App {
    var body: some Scene {
        WindowGroup("Stitch") {
            ContentView()
                .onAppear {
                    // When launched from a bare SwiftPM binary (not a bundle),
                    // make sure we come to the front like a real app.
                    NSApp.setActivationPolicy(.regular)
                    NSApp.activate(ignoringOtherApps: true)
                }
        }
        // Wide enough that a typical 3:1 panorama preview is legible on
        // first launch; the window resizes freely after that.
        .defaultSize(width: 1000, height: 640)
    }
}
