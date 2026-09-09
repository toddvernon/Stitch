import AppKit
import StitchCore
import SwiftUI
import UniformTypeIdentifiers

// The whole UI: a drop target that shows one of four screens depending on
// the model's phase. Input comes in by drag-and-drop or the file importer;
// output leaves through the save panel in PanoramaPane. Nothing here knows
// about the pipeline beyond the two settings pickers.

/// Root view. Owns the model and switches screens on its phase; the drop
/// destination and importer are attached at this level so they work in
/// every phase (a drop on the results screen starts a new stitch).
struct ContentView: View {
    @StateObject private var model = StitchModel()
    @State private var showImporter = false

    var body: some View {
        Group {
            switch model.phase {
            case .idle:
                dropPrompt
            case .running:
                progressView
            case .done:
                resultsView
            case .failed(let message):
                failureView(message)
            }
        }
        .frame(minWidth: 720, minHeight: 480)
        // Finder drops arrive as URLs; folders and files both go to the
        // model, which resolves them the same way the CLI does.
        .dropDestination(for: URL.self) { urls, _ in
            model.stitch(dropped: urls)
            return true
        }
        // Keyboard and menu path to the same thing, for people who do not
        // drag. Folders and images can be mixed in one selection.
        .fileImporter(isPresented: $showImporter,
                      allowedContentTypes: [.folder, .image],
                      allowsMultipleSelection: true) { result in
            if case .success(let urls) = result {
                model.stitch(dropped: urls)
            }
        }
    }

    // MARK: - States

    /// Idle: the drop target with the two pickers. Projection only applies
    /// to the rotational model, so it is disabled when Strip is forced;
    /// in Auto it is left enabled and simply ignored if a strip wins.
    private var dropPrompt: some View {
        VStack(spacing: 16) {
            Image(systemName: "photo.on.rectangle.angled")
                .font(.system(size: 56))
                .foregroundStyle(.secondary)
            Text("Drop a folder of photos to stitch")
                .font(.title2)
            Text("Photos are matched automatically — no ordering needed.\nJunk shots are ignored; multiple panoramas are recognized separately.\nA walk along a row of houses becomes a strip instead of a panorama.")
                .multilineTextAlignment(.center)
                .foregroundStyle(.secondary)
            Button("Choose Photos…") { showImporter = true }
                .keyboardShortcut("o")
            Picker("Mode", selection: $model.mode) {
                Text("Auto").tag(Stitcher.Mode.auto)
                Text("Panorama").tag(Stitcher.Mode.panorama)
                Text("Strip").tag(Stitcher.Mode.strip)
            }
            .pickerStyle(.segmented)
            .frame(maxWidth: 380)
            .padding(.top, 8)
            Picker("Projection", selection: $model.projection) {
                Text("Auto").tag(PanoProjection?.none)
                Text("Pannini").tag(PanoProjection?.some(.pannini))
                Text("Cylindrical").tag(PanoProjection?.some(.cylindrical))
                Text("Spherical").tag(PanoProjection?.some(.spherical))
            }
            .pickerStyle(.segmented)
            .frame(maxWidth: 380)
            .disabled(model.mode == .strip)
        }
        .padding(40)
    }

    /// Running: the pipeline's progress lines in a monospaced log that
    /// follows the newest entry, so a long strip render shows it is alive.
    private var progressView: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack(spacing: 10) {
                ProgressView()
                    .controlSize(.small)
                Text("Stitching…")
                    .font(.headline)
            }
            ScrollViewReader { proxy in
                ScrollView {
                    VStack(alignment: .leading, spacing: 2) {
                        ForEach(Array(model.log.enumerated()), id: \.offset) { i, line in
                            Text(line)
                                .font(.system(.caption, design: .monospaced))
                                .foregroundStyle(.secondary)
                                .id(i)
                        }
                    }
                    .frame(maxWidth: .infinity, alignment: .leading)
                }
                .onChange(of: model.log.count) {
                    proxy.scrollTo(model.log.count - 1, anchor: .bottom)
                }
            }
        }
        .padding(24)
    }

    /// Done: the outputs, plus the way back to idle.
    private var resultsView: some View {
        VStack(spacing: 0) {
            TabViewOrSingle(results: model.results)
            Divider()
            HStack {
                Button("Stitch Another…") { model.reset() }
                Spacer()
            }
            .padding(12)
        }
    }

    /// Failed: the model's message (already user-facing) and a retry.
    private func failureView(_ message: String) -> some View {
        VStack(spacing: 16) {
            Image(systemName: "exclamationmark.triangle")
                .font(.system(size: 44))
                .foregroundStyle(.orange)
            Text(message)
                .multilineTextAlignment(.center)
            Button("Try Again") { model.reset() }
        }
        .padding(40)
    }
}

/// Single panorama fills the window; multiple get a tab per panorama.
private struct TabViewOrSingle: View {
    let results: [StitchModel.PanoResult]

    var body: some View {
        if results.count == 1 {
            PanoramaPane(result: results[0])
        } else {
            TabView {
                ForEach(Array(results.enumerated()), id: \.element.id) { i, result in
                    PanoramaPane(result: result)
                        .tabItem { Text("\(result.kindName) \(i + 1)") }
                }
            }
            .padding(8)
        }
    }
}

/// One output: the preview scaled to fit, its caption, and Export. The
/// preview is the downscaled copy; export writes the full-resolution one.
private struct PanoramaPane: View {
    let result: StitchModel.PanoResult

    var body: some View {
        VStack(spacing: 10) {
            Image(decorative: result.preview, scale: 1)
                .resizable()
                .scaledToFit()
                .clipShape(RoundedRectangle(cornerRadius: 6))
                .shadow(radius: 4)
                .frame(maxWidth: .infinity, maxHeight: .infinity)
            HStack {
                Text(result.info)
                    .font(.caption)
                    .foregroundStyle(.secondary)
                Spacer()
                Button("Export…") { export(result) }
                    .keyboardShortcut("s")
            }
        }
        .padding(16)
    }

    /// Save panel, then the same writer the CLI uses, which picks the
    /// format from the extension. JPEG is the default name because the
    /// outputs are large and it is what gets shared; PNG and TIFF are a
    /// rename away. Write errors surface as a standard alert.
    private func export(_ result: StitchModel.PanoResult) {
        let panel = NSSavePanel()
        panel.allowedContentTypes = [.png, .jpeg, .tiff]
        panel.nameFieldStringValue = "\(result.suggestedName).jpg"
        panel.canCreateDirectories = true
        guard panel.runModal() == .OK, let url = panel.url else { return }
        do {
            try ImageLoader.writeImage(result.full, to: url)
        } catch {
            NSAlert(error: error).runModal()
        }
    }
}
