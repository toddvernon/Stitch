import Foundation

/// Dinic max-flow / min-cut on a directed graph, written in-house so the core
/// stays MIT-licensable (the well-known BK-maxflow code is research-only; the
/// algorithm itself is free). Used for graph-cut seam finding.
public final class MaxFlow {
    private let nodeCount: Int
    public let source: Int
    public let sink: Int

    // Edge arrays: edge i has twin i^1.
    private var edgeTo: [Int32] = []
    private var edgeCap: [Double] = []
    private var head: [Int32]        // per node: first edge index or -1
    private var next: [Int32] = []   // per edge: next edge index or -1

    private var level: [Int32]
    private var iter: [Int32]

    public init(nodeCount: Int) {
        self.nodeCount = nodeCount
        source = nodeCount
        sink = nodeCount + 1
        head = [Int32](repeating: -1, count: nodeCount + 2)
        level = [Int32](repeating: -1, count: nodeCount + 2)
        iter = [Int32](repeating: -1, count: nodeCount + 2)
    }

    /// Adds a directed edge u→v with capacity `cap` and reverse capacity `capRev`.
    public func addEdge(_ u: Int, _ v: Int, cap: Double, capRev: Double = 0) {
        addHalfEdge(u, v, cap)
        addHalfEdge(v, u, capRev)
    }

    public func addSourceEdge(_ u: Int, cap: Double) {
        addEdge(source, u, cap: cap)
    }

    public func addSinkEdge(_ u: Int, cap: Double) {
        addEdge(u, sink, cap: cap)
    }

    private func addHalfEdge(_ u: Int, _ v: Int, _ cap: Double) {
        edgeTo.append(Int32(v))
        edgeCap.append(cap)
        next.append(head[u])
        head[u] = Int32(edgeTo.count - 1)
    }

    @discardableResult
    public func solve() -> Double {
        var flow = 0.0
        while bfs() {
            iter = head
            while true {
                let f = dfs(from: source, limit: .infinity)
                if f <= 0 { break }
                flow += f
            }
        }
        return flow
    }

    /// After solve(): true if the node is on the source side of the min cut.
    public func isSourceSide(_ u: Int) -> Bool {
        level[u] >= 0
    }

    private func bfs() -> Bool {
        for i in level.indices { level[i] = -1 }
        var queue = [Int32(source)]
        level[source] = 0
        var qi = 0
        while qi < queue.count {
            let u = Int(queue[qi])
            qi += 1
            var e = head[u]
            while e >= 0 {
                let v = Int(edgeTo[Int(e)])
                if edgeCap[Int(e)] > 1e-12, level[v] < 0 {
                    level[v] = level[u] + 1
                    queue.append(Int32(v))
                }
                e = next[Int(e)]
            }
        }
        return level[sink] >= 0
    }

    /// Iterative blocking-flow DFS (explicit stack; grids are large).
    private func dfs(from start: Int, limit: Double) -> Double {
        var path: [Int32] = []          // edges along the current path
        var node = start
        while true {
            if node == sink {
                var bottleneck = Double.infinity
                for e in path { bottleneck = min(bottleneck, edgeCap[Int(e)]) }
                for e in path {
                    edgeCap[Int(e)] -= bottleneck
                    edgeCap[Int(e) ^ 1] += bottleneck
                }
                return bottleneck
            }
            var advanced = false
            while iter[node] >= 0 {
                let e = Int(iter[node])
                let v = Int(edgeTo[e])
                if edgeCap[e] > 1e-12, level[v] == level[node] + 1 {
                    path.append(Int32(e))
                    node = v
                    advanced = true
                    break
                }
                iter[node] = next[e]
            }
            if advanced { continue }
            // Dead end: retreat.
            level[node] = -1
            guard let lastEdge = path.popLast() else { return 0 }
            // The tail of the last edge is where we came from.
            node = Int(edgeTo[Int(lastEdge) ^ 1])
            iter[node] = next[Int(lastEdge)]
        }
    }
}
