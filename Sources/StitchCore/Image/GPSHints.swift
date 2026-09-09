// Advisory use of EXIF GPS positions (DESIGN.md, "GPS is advisory, never
// required"). Feeds a log line, an explanation for unplaced photos, and the
// auto-mode tiebreak in `Stitcher`. Positions never enter the geometry.

import Foundation

/// A WGS-84 position in signed decimal degrees (south and west negative).
public struct GPSCoordinate: Equatable, Sendable {
    public var latitude: Double
    public var longitude: Double

    public init(latitude: Double, longitude: Double) {
        self.latitude = latitude
        self.longitude = longitude
    }

    /// Ground distance in meters (equirectangular approximation, plenty for
    /// the tens-to-hundreds of meters between shots).
    public func distance(to other: GPSCoordinate) -> Double {
        let r = 6_371_000.0   // mean Earth radius, meters
        let lat1 = latitude * .pi / 180, lat2 = other.latitude * .pi / 180
        let dLat = lat2 - lat1
        let dLon = (other.longitude - longitude) * .pi / 180 * cos((lat1 + lat2) / 2)
        return r * (dLat * dLat + dLon * dLon).squareRoot()
    }
}

/// What photo positions can tell us, and no more. Phone GPS is good to a
/// few meters in the open, so it can say whether the photographer walked
/// and roughly how far between shots: enough to explain a photo that
/// didn't connect, and to break a tie in auto mode, but nothing about
/// alignment, which the image content settles far more precisely. Every
/// entry point tolerates missing positions; with none present, every
/// result is nil or empty.
///
/// Photos are identified by their index into the caller's URL list, so
/// a sparse dictionary represents "some photos have positions".
public enum GPSHints {

    /// Positions this far apart are beyond GPS noise: the photographer moved.
    /// Phone fixes scatter by 5 to 10 m even standing still, so 15 m is the
    /// smallest span that reads as a deliberate step.
    public static let movedThreshold = 15.0

    /// Reads the EXIF position of each URL; absent ones are simply omitted.
    public static func coordinates(urls: [URL]) -> [Int: GPSCoordinate] {
        var result: [Int: GPSCoordinate] = [:]
        for (i, url) in urls.enumerated() {
            if let c = ImageLoader.gpsCoordinate(url: url) { result[i] = c }
        }
        return result
    }

    /// Largest distance between any two positioned photos, nil unless at
    /// least two have positions.
    public static func spread(_ coords: [Int: GPSCoordinate]) -> Double? {
        let cs = Array(coords.values)
        guard cs.count >= 2 else { return nil }
        var best = 0.0
        for i in cs.indices {
            for j in (i + 1)..<cs.count {
                best = max(best, cs[i].distance(to: cs[j]))
            }
        }
        return best
    }

    /// Distance from each positioned photo to its nearest positioned neighbor.
    public static func nearestNeighborDistances(_ coords: [Int: GPSCoordinate]) -> [Int: Double] {
        var result: [Int: Double] = [:]
        for (i, a) in coords {
            var best = Double.infinity
            for (j, b) in coords where j != i {
                best = min(best, a.distance(to: b))
            }
            if best.isFinite { result[i] = best }
        }
        return result
    }

    /// Median nearest-neighbor distance: the photographer's typical step.
    /// The median ignores one dropped frame's large gap, which is exactly
    /// the outlier `gapNotes` wants to measure against.
    public static func typicalStep(_ coords: [Int: GPSCoordinate]) -> Double? {
        let d = nearestNeighborDistances(coords).values.sorted()
        guard !d.isEmpty else { return nil }
        return d[d.count / 2]
    }

    /// One-line summary for the log, or nil without positions.
    public static func summary(_ coords: [Int: GPSCoordinate], totalPhotos: Int) -> String? {
        guard let spread = spread(coords), let step = typicalStep(coords) else { return nil }
        let coverage = coords.count < totalPhotos ? " (\(coords.count) of \(totalPhotos) photos positioned)" : ""
        if spread < movedThreshold {
            return "GPS: all photos within \(Int(spread.rounded())) m, shot from one spot\(coverage)"
        }
        return "GPS: photos span \(Int(spread.rounded())) m, typical step \(Int(step.rounded())) m\(coverage)"
    }

    /// Explanations for photos that no group placed: which ones sit
    /// unusually far from any other photo, in terms of the typical step.
    public static func gapNotes(unplaced: [Int], coords: [Int: GPSCoordinate],
                                names: [String]) -> [String] {
        guard let step = typicalStep(coords), step > 0 else { return [] }
        let nearest = nearestNeighborDistances(coords)
        var notes: [String] = []
        for i in unplaced.sorted() {
            guard let d = nearest[i] else { continue }
            let ratio = d / step
            // 1.6× is beyond the spread of a steady walk but under the 2×
            // of a single missing frame, so one dropped shot is caught.
            if ratio >= 1.6 {
                notes.append("\(names[i]): \(Int(d.rounded())) m from the nearest photo, "
                             + "\(String(format: "%.1f", ratio))× the typical step — likely a missing frame, no overlap")
            }
        }
        return notes
    }
}
