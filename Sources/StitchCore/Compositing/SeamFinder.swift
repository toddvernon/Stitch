import Foundation

/// Graph-cut seam finding (Kwatra 2003 / Agarwala 2004, as used in stitching
/// pipelines): assigns every pano pixel to exactly one source image, with cut
/// boundaries routed through low-difference pixels so moving objects and
/// residual parallax come entirely from a single photo.
public enum SeamFinder {

    /// Returns a label map (geometry-sized, -1 where no image covers the pixel)
    /// built by sequential pairwise binary cuts, one new image at a time.
    public static func labels(layers: [ImageLayer], width: Int, height: Int) -> [Int32] {
        var label = [Int32](repeating: -1, count: width * height)
        var compositeIntensity = [Float](repeating: 0, count: width * height)

        func intensity(_ layer: ImageLayer, _ localIndex: Int) -> Float {
            (layer.rgb.r.pixels[localIndex] + layer.rgb.g.pixels[localIndex] + layer.rgb.b.pixels[localIndex]) / 3
        }

        for layer in layers {
            let k = Int32(layer.imageIndex)
            var overlap: [Int] = []          // pano indices covered by both
            var nodeOf = [Int32](repeating: -1, count: width * height)

            for row in 0..<layer.height {
                let py = layer.y0 + row
                for col in 0..<layer.width {
                    let li = row * layer.width + col
                    guard layer.validity.pixels[li] > 0.5 else { continue }
                    let pi = py * width + (layer.x0 + col)
                    if label[pi] < 0 {
                        label[pi] = k
                        compositeIntensity[pi] = intensity(layer, li)
                    } else {
                        nodeOf[pi] = Int32(overlap.count)
                        overlap.append(pi)
                    }
                }
            }
            guard !overlap.isEmpty else { continue }

            // New-image intensity over the overlap.
            var newIntensity = [Float](repeating: 0, count: overlap.count)
            for (node, pi) in overlap.enumerated() {
                let py = pi / width, px = pi % width
                let li = (py - layer.y0) * layer.width + (px - layer.x0)
                newIntensity[node] = intensity(layer, li)
            }

            let flow = MaxFlow(nodeCount: overlap.count)
            let big = 1e9
            for (node, pi) in overlap.enumerated() {
                let py = pi / width, px = pi % width
                let diffHere = Double(abs(compositeIntensity[pi] - newIntensity[node]))

                for (nx, ny) in [(px + 1, py), (px, py + 1), (px - 1, py), (px, py - 1)] {
                    guard nx >= 0, nx < width, ny >= 0, ny < height else { continue }
                    let qi = ny * width + nx
                    let qNode = nodeOf[qi]
                    if qNode >= 0 {
                        // Overlap neighbor: smoothness edge (add once per pair).
                        if qi > pi {
                            let diffThere = Double(abs(compositeIntensity[qi] - newIntensity[Int(qNode)]))
                            let wEdge = diffHere + diffThere + 1e-4
                            flow.addEdge(node, Int(qNode), cap: wEdge, capRev: wEdge)
                        }
                    } else if label[qi] >= 0, label[qi] != k {
                        // Adjacent to existing-composite-only territory.
                        flow.addSourceEdge(node, cap: big)
                    } else if label[qi] == k {
                        // Adjacent to new-image-only territory.
                        flow.addSinkEdge(node, cap: big)
                    }
                }
            }
            flow.solve()

            for (node, pi) in overlap.enumerated() where !flow.isSourceSide(node) {
                label[pi] = k
                compositeIntensity[pi] = newIntensity[node]
            }
        }
        return label
    }

    /// Binary mask (1/0) for one image from the label map, at label resolution.
    public static func mask(for imageIndex: Int, labels: [Int32], width: Int, height: Int) -> ImageF {
        var m = ImageF(width: width, height: height)
        let k = Int32(imageIndex)
        for i in 0..<(width * height) where labels[i] == k {
            m.pixels[i] = 1
        }
        return m
    }
}
