// Compositing stage 9, the solver under SeamFinder. A binary pixel labeling
// is a min cut: source side keeps the composite, sink side takes the new
// image, terminal edges carry the data costs and neighbor edges the
// smoothness costs (Boykov & Jolly 2001 formulation).

import Foundation

/// Dinic max-flow / min-cut on a directed graph, written in-house so the core
/// stays MIT-licensable (the well-known BK-maxflow code is research-only; the
/// algorithm itself is free). Used for graph-cut seam finding.
///
/// Dinic (1970) runs in phases: a BFS builds a level graph over the residual
/// network, then a DFS pushes a blocking flow along shortest augmenting
/// paths. Worst case O(V²E), but on the nearly planar grid graphs seams
/// produce it is fast enough that the working-resolution cap in
/// `Compositor.Options.seamWidth` is what bounds the cost, not the solver.
/// Boykov-Kolmogorov would be quicker on these graphs; this is the simplest
/// correct algorithm with a free license.
public final class MaxFlow {
    private let nodeCount: Int
    /// Terminal node ids: the caller's nodes are 0..<nodeCount, and the two
    /// terminals are appended after them.
    public let source: Int
    public let sink: Int

    // Adjacency as parallel arrays with per-node linked lists (compact and
    // cache-friendly for millions of edges). Edge i has twin i^1: every
    // addEdge appends the forward and reverse half-edges back to back, so a
    // residual push on i is undone on i^1 with one XOR.
    private var edgeTo: [Int32] = []
    private var edgeCap: [Double] = []
    private var head: [Int32]        // per node: first edge index or -1
    private var next: [Int32] = []   // per edge: next edge index or -1

    // Dinic state: BFS level per node (-1 = unreached), and the current-arc
    // pointer per node so each DFS phase scans each edge list once.
    private var level: [Int32]
    private var iter: [Int32]

    /// Graph over `nodeCount` caller nodes plus source and sink.
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

    /// Terminal edge source→u: the cost of putting `u` on the sink side.
    public func addSourceEdge(_ u: Int, cap: Double) {
        addEdge(source, u, cap: cap)
    }

    /// Terminal edge u→sink: the cost of putting `u` on the source side.
    public func addSinkEdge(_ u: Int, cap: Double) {
        addEdge(u, sink, cap: cap)
    }

    private func addHalfEdge(_ u: Int, _ v: Int, _ cap: Double) {
        edgeTo.append(Int32(v))
        edgeCap.append(cap)
        next.append(head[u])
        head[u] = Int32(edgeTo.count - 1)
    }

    /// Runs Dinic to completion and returns the max-flow value (equal to the
    /// min-cut cost). Afterwards `isSourceSide` reads the cut.
    @discardableResult
    public func solve() -> Double {
        var flow = 0.0
        while bfs() {
            // Reset current-arc pointers for this phase, then drain the
            // level graph: repeated DFS until no augmenting path remains.
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
    /// The final BFS (the one that failed to reach the sink) leaves `level`
    /// set exactly on the nodes still reachable from the source in the
    /// residual graph, which is the source side by the max-flow/min-cut
    /// theorem.
    public func isSourceSide(_ u: Int) -> Bool {
        level[u] >= 0
    }

    /// Builds the level graph over residual edges. Returns false when the
    /// sink is unreachable, which ends the algorithm.
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
                // Capacities are sums of float intensity differences, so
                // anything below 1e-12 is rounding noise, not residual.
                if edgeCap[Int(e)] > 1e-12, level[v] < 0 {
                    level[v] = level[u] + 1
                    queue.append(Int32(v))
                }
                e = next[Int(e)]
            }
        }
        return level[sink] >= 0
    }

    /// Iterative blocking-flow DFS (explicit stack; grids are large). Finds
    /// one augmenting path in the level graph, pushes its bottleneck, and
    /// returns the amount; 0 when the level graph is exhausted. `iter` is the
    /// current-arc optimization: an edge that failed once in this phase is
    /// never rescanned.
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
            // Dead end: retreat. Dropping the node out of the level graph is
            // safe because nothing can reach the sink through it this phase.
            level[node] = -1
            guard let lastEdge = path.popLast() else { return 0 }
            // The tail of the last edge is where we came from.
            node = Int(edgeTo[Int(lastEdge) ^ 1])
            iter[node] = next[Int(lastEdge)]
        }
    }
}
