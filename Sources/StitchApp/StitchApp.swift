import AppKit
import SwiftUI

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
        .defaultSize(width: 1000, height: 640)
    }
}
