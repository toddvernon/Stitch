import Foundation
import simd

// Stage 5 of the pipeline (DESIGN.md): joint refinement of every placed
// camera's rotation and focal length. PanoramaAligner builds the
// observations and calls `adjust` after each image it places; the dense
// Cholesky solver at the bottom is shared with MeshRefiner and StripAligner.

/// One correspondence used by bundle adjustment: the ray of `pointB` in camera
/// B should project to `pointA` in camera A (centered pixel coordinates).
/// Callers add each feature match in both directions.
public struct MatchObservation {
    /// Indices into the `cameras` array handed to `adjust`.
    public var cameraA: Int
    public var cameraB: Int
    /// Centered pixel coordinates (see `Camera`), registration scale.
    public var pointA: SIMD2<Double>
    public var pointB: SIMD2<Double>

    public init(cameraA: Int, cameraB: Int, pointA: SIMD2<Double>, pointB: SIMD2<Double>) {
        self.cameraA = cameraA
        self.cameraB = cameraB
        self.pointA = pointA
        self.pointB = pointB
    }
}

/// Levenberg-Marquardt bundle adjustment over rotations and focal lengths
/// (Brown & Lowe IJCV 2007 §4): 4 parameters per camera, analytic Jacobians,
/// Huber-robustified reprojection error, and the paper's prior covariance
/// (σ_θ = π/16, σ_f = f̄/10) as the damping matrix.
public enum BundleAdjuster {

    /// Optimizes `cameras` in place. `huberSigma` nil means plain L2 (the
    /// paper's initialization schedule); 2.0 is the final robust pass.
    /// Returns the final RMS reprojection error in pixels over all residuals.
    @discardableResult
    public static func adjust(cameras: inout [Camera],
                              observations: [MatchObservation],
                              huberSigma: Double? = nil,
                              maxIterations: Int = 30) -> Double {
        let nCams = cameras.count
        let nParams = nCams * 4
        guard nCams >= 2, !observations.isEmpty else { return rms(cameras, observations) }

        // LM damping factor. It scales the prior precision (below) rather
        // than the identity, so a rotation step and a focal step are damped
        // in comparable units.
        var lambda = 1e-3
        var error = totalError(cameras, observations, huberSigma)

        for _ in 0..<maxIterations {
            var a = [Double](repeating: 0, count: nParams * nParams)
            var g = [Double](repeating: 0, count: nParams)
            accumulateNormalEquations(cameras, observations, huberSigma, into: &a, gradient: &g)

            // Paper §4.1: prior covariance C_p with σ_θ = π/16 rad and
            // σ_f = f̄/10; the damped system is (JᵀWJ + λ·C_p⁻¹) δ = −JᵀWe.
            let fMean = cameras.map(\.focal).reduce(0, +) / Double(nCams)
            let sigmaTheta = Double.pi / 16
            let sigmaF = fMean / 10

            // Up to 8 damping increases per outer iteration before giving up.
            var improved = false
            for _ in 0..<8 {
                var damped = a
                for c in 0..<nCams {
                    for k in 0..<3 {
                        let i = c * 4 + k
                        damped[i * nParams + i] += lambda / (sigmaTheta * sigmaTheta)
                    }
                    let i = c * 4 + 3
                    damped[i * nParams + i] += lambda / (sigmaF * sigmaF)
                }
                guard let delta = choleskySolve(damped, negated(g), n: nParams) else {
                    lambda *= 10
                    continue
                }
                // Apply the step. The rotation update is right-multiplied
                // (in the camera's own frame), matching how the Jacobian is
                // built in `accumulateNormalEquations`. Focal floored at 10 px
                // so a wild step can't drive it through zero.
                var trial = cameras
                for c in 0..<nCams {
                    let dTheta = SIMD3(delta[c * 4], delta[c * 4 + 1], delta[c * 4 + 2])
                    trial[c].rotation = trial[c].rotation * SO3.exp(dTheta)
                    trial[c].focal = max(trial[c].focal + delta[c * 4 + 3], 10)
                }
                // Marquardt's schedule: accept and ease damping toward
                // Gauss-Newton, or reject and damp harder toward gradient
                // descent. Stop once an accepted step barely moves the error.
                let trialError = totalError(trial, observations, huberSigma)
                if trialError < error {
                    cameras = trial
                    let relative = (error - trialError) / max(error, 1e-12)
                    error = trialError
                    lambda = max(lambda * 0.3, 1e-9)
                    improved = true
                    if relative < 1e-6 { return rms(cameras, observations) }
                    break
                } else {
                    lambda *= 10
                }
            }
            if !improved { break }
        }
        return rms(cameras, observations)
    }

    // MARK: - Residuals

    /// Prediction error of observation `o`: projection of B's ray into A,
    /// minus the observed A pixel. Nil when the ray is behind camera A.
    static func residual(_ cams: [Camera], _ o: MatchObservation) -> SIMD2<Double>? {
        let ca = cams[o.cameraA], cb = cams[o.cameraB]
        let qB = SIMD3(o.pointB.x / cb.focal, o.pointB.y / cb.focal, 1)
        let world = cb.rotation.transpose * qB
        let p = ca.rotation * world
        guard p.z > 1e-9 else { return nil }
        return SIMD2(ca.focal * p.x / p.z - o.pointA.x,
                     ca.focal * p.y / p.z - o.pointA.y)
    }

    /// Huber loss (paper eq. 17); σ = nil is plain L2. Quadratic inside σ,
    /// linear outside, continuous at the knee.
    private static func huberLoss(_ normSq: Double, _ sigma: Double?) -> Double {
        guard let sigma else { return normSq }
        let n = sqrt(normSq)
        return n < sigma ? normSq : 2 * sigma * n - sigma * sigma
    }

    /// The robustified objective (paper eq. 16). A ray that ends up behind
    /// camera A costs a flat large penalty, so a step that swings a camera
    /// around backwards is always rejected.
    private static func totalError(_ cams: [Camera], _ obs: [MatchObservation], _ sigma: Double?) -> Double {
        var e = 0.0
        let behindPenalty = 1e6
        for o in obs {
            if let r = residual(cams, o) {
                e += huberLoss(length_squared(r), sigma)
            } else {
                e += behindPenalty
            }
        }
        return e
    }

    /// Unrobustified RMS reprojection error, px, over observations in front
    /// of the camera. This is the figure callers report.
    private static func rms(_ cams: [Camera], _ obs: [MatchObservation]) -> Double {
        var sum = 0.0
        var count = 0
        for o in obs {
            if let r = residual(cams, o) {
                sum += length_squared(r)
                count += 1
            }
        }
        return count > 0 ? sqrt(sum / Double(count)) : .infinity
    }

    // MARK: - Normal equations

    /// One Gauss-Newton linearization over all observations: accumulates
    /// JᵀWJ into `a` and JᵀWe into `g` (paper §4.1, eqs. 18 to 21), W being
    /// the IRLS Huber weight. Only the eight parameters of the two cameras in
    /// an observation touch its rows, so each one adds an 8x8 block.
    private static func accumulateNormalEquations(_ cams: [Camera],
                                                  _ obs: [MatchObservation],
                                                  _ sigma: Double?,
                                                  into a: inout [Double],
                                                  gradient g: inout [Double]) {
        let nParams = cams.count * 4

        for o in obs {
            let ca = cams[o.cameraA], cb = cams[o.cameraB]
            let qB = SIMD3(o.pointB.x / cb.focal, o.pointB.y / cb.focal, 1)
            let world = cb.rotation.transpose * qB
            let p = ca.rotation * world
            guard p.z > 1e-9 else { continue }

            let e = SIMD2(ca.focal * p.x / p.z - o.pointA.x,
                          ca.focal * p.y / p.z - o.pointA.y)

            // IRLS weight for the Huber loss.
            var w = 1.0
            if let sigma {
                let n = length(e)
                if n > sigma { w = sigma / n }
            }

            // ∂(projected pixel)/∂p, 2x3 (paper eq. 21 with focal folded in).
            let fz = ca.focal / p.z
            let dproj = [
                SIMD3(fz, 0, -fz * p.x / p.z),
                SIMD3(0, fz, -fz * p.y / p.z),
            ]

            // Columns of the 2x8 Jacobian: [θA(3), fA, θB(3), fB].
            // ∂p/∂θ_A: rotating camera A by a small angle about generator k
            // (right-multiplied, as in `adjust`) moves p by R_A·G_k·world.
            // Camera B's rotation enters through `world` with the same
            // generator and the opposite sign.
            var jac = [SIMD2<Double>](repeating: .zero, count: 8)
            for k in 0..<3 {
                let dpA = ca.rotation * (SO3.generators[k] * world)
                jac[k] = SIMD2(dot(dproj[0], dpA), dot(dproj[1], dpA))
                jac[4 + k] = -jac[k]
            }
            // ∂pixel/∂f_A = p/z, since the pixel is f·p/z. For f_B, the ray
            // q_B = (x/f_B, y/f_B, 1) has ∂q_B/∂f_B = −(q_B.x, q_B.y, 0)/f_B.
            jac[3] = SIMD2(p.x / p.z, p.y / p.z)
            let dqB = SIMD3(-qB.x / cb.focal, -qB.y / cb.focal, 0)
            let dpB = ca.rotation * (cb.rotation.transpose * dqB)
            jac[7] = SIMD2(dot(dproj[0], dpB), dot(dproj[1], dpB))

            // Scatter the upper triangle of the 8x8 block into the full
            // symmetric matrix; the guard keeps diagonal entries from being
            // added twice.
            let offsets = [o.cameraA * 4, o.cameraA * 4 + 1, o.cameraA * 4 + 2, o.cameraA * 4 + 3,
                           o.cameraB * 4, o.cameraB * 4 + 1, o.cameraB * 4 + 2, o.cameraB * 4 + 3]
            for r in 0..<8 {
                let jr = jac[r]
                g[offsets[r]] += w * dot(jr, e)
                for c in r..<8 {
                    let v = w * dot(jr, jac[c])
                    a[offsets[r] * nParams + offsets[c]] += v
                    if offsets[r] != offsets[c] {
                        a[offsets[c] * nParams + offsets[r]] += v
                    }
                }
            }
        }
    }

    // MARK: - Linear algebra

    private static func negated(_ v: [Double]) -> [Double] {
        v.map { -$0 }
    }

    /// Dense Cholesky solve of A x = b for symmetric positive definite A.
    /// Returns nil when a pivot is not positive (A not positive definite, or
    /// numerically singular); the LM loop treats that as a failed step.
    /// O(n³), fine for the sizes here: 4 unknowns per camera, up to about a
    /// thousand for a warp mesh.
    static func choleskySolve(_ a: [Double], _ b: [Double], n: Int) -> [Double]? {
        var l = [Double](repeating: 0, count: n * n)
        for i in 0..<n {
            for j in 0...i {
                var sum = a[i * n + j]
                for k in 0..<j {
                    sum -= l[i * n + k] * l[j * n + k]
                }
                if i == j {
                    guard sum > 1e-12 else { return nil }
                    l[i * n + i] = sqrt(sum)
                } else {
                    l[i * n + j] = sum / l[j * n + j]
                }
            }
        }
        // Forward substitution: L y = b
        var y = [Double](repeating: 0, count: n)
        for i in 0..<n {
            var sum = b[i]
            for k in 0..<i {
                sum -= l[i * n + k] * y[k]
            }
            y[i] = sum / l[i * n + i]
        }
        // Back substitution: Lᵀ x = y
        var x = [Double](repeating: 0, count: n)
        for i in stride(from: n - 1, through: 0, by: -1) {
            var sum = y[i]
            for k in (i + 1)..<n {
                sum -= l[k * n + i] * x[k]
            }
            x[i] = sum / l[i * n + i]
        }
        return x
    }
}
