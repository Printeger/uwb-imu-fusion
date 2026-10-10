# Isolated UWB–IMU FDE research, 2026-10-10

The bounded original-sensor convergence task is complete; see
[functional results and refusal boundaries](../docs/fde_functional_convergence.md).
It retains the evidence below, adds two six-axis motion snapshots and actual
UWB exclusion-lineage evaluation, and does not promote the odometer comparison
to an original UWB+IMU positive. `fde_research_power CONFIG OUTPUT directions`
executes the new directed checks. `report_convergence.py` validates the retained
small runs and appends `functional_convergence` to the existing JSON.

Production remains **PARTIAL_BLOCKED**. These `EXCLUDE_FROM_ALL` targets are
physically separate from the production library/node, are not installed, and
never retain/mint a protected output or call a publisher. Simulation results
always have `formal_eligible=false`, `publication_protected=false`. No production
configuration, probability, detector threshold, step/rank gate or Huber code was
changed. The accepted persistent-constant merging counterexample is retained;
that experiment was not repeated.

## Frozen inputs and commands

Use `config/fde_imu_order1.yaml` (SHA256
`7feddddb68dc3872d19435c21538bbf8ab61c5316fc57dfcc23d056caf5dceee`). The research
contract below is compiled into the isolated fixture; it is **not** a replacement
YAML loaded by the production entry. Old requirements/progress evidence remains
intact. Commands assume this workspace and its existing catkin build:

```sh
cmake -S . -B /home/mint/ws_fusion_uwb/build/uwb_imu_pl
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target fde_research_support fde_research_prior fde_research_power fde_research_recovery fde_research_contract_checks fde_research_cost -j2
export LD_LIBRARY_PATH=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib
fde_research_bin=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl
"$fde_research_bin/fde_research_support" config/fde_imu_order1.yaml /tmp/support.csv
"$fde_research_bin/fde_research_prior"
"$fde_research_bin/fde_research_contract_checks" config/fde_imu_order1.yaml
"$fde_research_bin/fde_research_power" config/fde_imu_order1.yaml /tmp/power.csv
"$fde_research_bin/fde_research_recovery" config/fde_imu_order1.yaml /tmp/normal.csv normal 0
"$fde_research_bin/fde_research_recovery" config/fde_imu_order1.yaml /tmp/uwb.csv uwb 0
"$fde_research_bin/fde_research_recovery" config/fde_imu_order1.yaml /tmp/imu.csv imu 1
```

The last argument adds an explicitly simulated independent odometer. `imu 0`
retains the insufficient-information negative case. GT produces the raw sensor
samples and evaluates committed position only; no fault label enters the model
registry, evidence evaluator, action generator, selector or transaction.

## A: physical raw episode and history

An episode is `(sensor, body axis, [begin,end), amplitude f)`, with support
`s_k=1{begin <= t_k < end}` on actual raw sample IDs/timestamps. For trapezoid k,
input perturbation is `u_k f`, where `u_k=(s_(k-1)+s_k)/2`. The ten new 5 ms samples
from 5 through 50 ms give first-factor acceleration responses
`delta_p/f=0.00113125 s²`, `delta_v/f=0.0475 s`. The shared last sample also gives
next-factor `0.00011875 s²`, `0.0025 s`. It is not an interval bias rescaling.

`integrateEpisode` copies the real PIM parameters and **biasHat**, then integrates
actual body acceleration/angular rate and actual time differences. GTSAM handles
rotation, bias correction, preintegration Jacobians and its covariance recursion.
At frozen endpoint poses, velocities and biases, define real Combined residual
`e_j(f)`. The signal column is `D_j=-d e_j/df`, validated by central differences,
half-step convergence and an independent raw-corruption/reintegration oracle.
The implementation separately evaluates `-d[R_j(f)e_j(f)]/df` and `dQ_j/df`;
using frozen `R_j D_j` does not assert that input-dependent covariance disappears.
Static/rotating cases, six axes, nonzero endpoint biases/bias drift, adjacent
factors and absent support are deterministic checks.

Stack both factor responses under **one** episode parameter before elimination.
For old/boundary states, orthogonal elimination of
`H_o x_o + H_b x_b + D f - z` gives `R_b x_b + T_b f - d_b` and
`F_b f - d_perp`. The sum of squared norms equals the independently minimized
original cost. Shared-parameter tests verify this for both factors together, not
just for two separately charged interval faults. A removed factor does not
remove the last bad raw sample: the recovery registry reconstructs latent carry
modes from raw history, without looking at the injected truth event.

There is also a separate noise issue. If raw sample k has covariance S_k and
Combined error Jacobians J_jk, the joint covariance includes
`Cov(e_j,e_l)=sum_k J_jk S_k J_lk'`. At a shared sample this is generally nonzero.
Actual-factor FD tests form the joint PSD covariance for a declared research ADC
noise law (`sigma_accel=.02 m/s²`, `sigma_gyro=.001 rad/s`) and verify nonzero
cross blocks. This law is not hardware calibration. The generic orthogonal
history operation must transform the joint covariance with the same row
operator; independent marginal PIM covariances are insufficient to infer it.
The production history/noise contract has not been silently changed.

## B: detection and limits

With frozen independent Gaussian whitened errors, `Q2` spans parity and
`Gamma=||Q2' D||²`, `lambda=f² Gamma`; the original threshold and dof are used.
The four directed windows compare duration, rotation, geometry and history.
At stationary epoch 12, accel-x 20 m/s² gives:

| Duration | lambda | conditional p_md |
| --- | ---: | ---: |
| 50 ms | 2.9593825623 | 0.9999965593 |
| 250 ms | 77.1511096711 | 0.5792341582 |
| 500 ms | 174.6448256304 | 0.0002367432 |

Longer observation of a persistent acceleration episode can meet the *power*
requirement but gives about `20*.3489=6.98` state step, above the unchanged .25
gate. It is not a recovery witness. At epoch 3, the observed 50 ms accel event
has statistic .0915054, far below the original 72.22885 threshold. Original
mature-stream evidence (.187 vs 121.3488; velocity absorption) is reused.
Stationary gyro-z has lambda approximately 1e-26 even for 500 ms: no yaw reference
is provided by stationary range geometry. Rotation/flat geometry effects are
axis-specific; changing anchor height is not universally helpful or harmful.

The nc-chi² numbers are **conditional frozen-model** results, not hardware miss
probabilities. For correlated raw noise Ω, the actual frozen linear Gaussian
statistic has `K=Q2'ΩQ2`, `mu=Q2'Df`:
`S=sum_i d_i (Z_i + mu_i/sqrt(d_i))²`, with degenerate directions handled by the
quadratic form. It is generalized, not ordinary nc-chi². `quadraticPower` computes
`E[S]=tr(K)+||mu||²`, `Var(S)=2tr(K²)+4mu'Kmu` and rigorous Cantelli miss bounds
under that declared linear Gaussian law. Actual two-factor fixtures verify that
shared sample noise changes this premise. No production detector was changed.
Hardware p_md remains UNQUALIFIED without the joint noise/process evidence.

## C: conditional closed-loop contract v1

`simulation-only/raw-range-episode/v1` is a complete categorical generation law
on a 350 ms, seven-epoch horizon: nominal, or exactly one `(epoch,anchor)` UWB
bias +2.25 m, or one accel-x raw batch +20 m/s². Each UWB atom has probability
1e-4, each IMU atom 1e-5; with eight anchors nominal mass is .99433. Distinct atoms
are mutually exclusive **by this generator**, not by production hardware
independence. Conditional witness flows do not estimate their frequency.
The registry conservatively retains six IMU axes and the original priors and
p_md allocations; equal allocation uses the existing conservative allocator.
No temporal source merging, lower prior or larger risk budget is used.

Physical assumptions: stationary body at (0,0,1), zero lever arm, zero true
acceleration/angular rate, exact generated ranges with deterministic error
bound 1e-12 m, at most one bad anchor per epoch. IMU inputs use gravity plus the
episode. With the independent odometer, raw measured velocity is zero in this
explicit zero-error realization; optimization weight is .02 m/s. Velocity states
remain variables. This is additional sensor information, not frozen velocity,
a motion prior claimed as an existing production sensor, or a fault label.
Without it the 50 ms negative case remains insufficiently detectable.

For each omit-none/omit-one range subset, difference-of-squared-range equations
are `A p=y`. Reject rank-unresolved geometry and inconsistent least-squares
orthogonal residuals. The difference-equation uncertainty and a numerical guard
give a position ball using the singular-value lower estimate; sphere residuals
provide a further necessary test. The union contains every realization of the
declared one-corrupt-range model. It uses raw ranges and anchors, never GT.
This is a prototype with guarded floating-point SVD, not a certified interval
arithmetic implementation for arbitrary scale/hardware data.

Choose common reference r from this same union. Its coordinate bound L_r is the
maximum over union balls. For **every** candidate a, the producer independently
checks `L_a=L_r+|p_a-r|`, HPL, VPL and exact raw source/time/model metadata. PL/centre
mutation, foreign model, malformed proof and cross-time source are refused.
A separate typed consumer check applies before the actual FdeManager for KEEP
and successful IMU paths too; logical proof IDs alone never provide this check.
A single deterministic accepted event implies these inequalities for **all**
candidates simultaneously; adaptive selection introduces no additional failure
event. This is a new simulation selector proof, not caller-set shared flags and
not a change to production singleton guarantee grouping. Its original strict
FAIL remains in each CSV's `strict_*` fields.

Under this generator the current range position guarantee is independent of
process/bridge/history/optimizer errors, so their conditional position escape
terms are structurally zero. At most one episode and exhaustive traversal prove
omitted/envelope zero. These are explicit model premises, not hardware
qualification booleans. The original conservative numerical ledger still charges
nominal 3e-5, p_nm 1e-7, hypothesis allocation 9.9e-6; all known miss terms are
retained, and selection-extra is computed by the research selector. Total is
4e-5 (original budget), even though this deterministic conditional position
bound has no HMI realization within its premises. These ledger amounts are
conservative HMI charges under the stronger range-set guarantee; they do not
prove that the configured .001 p_md is a calibrated raw-sensor miss probability. Full-precision fault/KEEP
ledgers are in the closure logs.

Both UWB and IMU fault streams automatically detect, generate/evaluate actions,
check original post detector and step/rank gates, select, and execute actual
iSAM2 transactions once per epoch. Each excludes at epoch 3 and continues four
selected risk-closing commits. The IMU selector conservatively removes groups
2001 and 3001 to cover its plausible set; it does not claim exact temporal
identification from an ambiguous profile. IMU uses a generic independent bridge and a
full current-position conditional bound, not BRIDGE_ONLY. After nonlinear commit
the range set is transferred again to the **actual committed mean**. Every
output remains unprotected. Fault processing exceeds 40/50 ms; finite late
results do not count as timely protection. Production unrecovered latency stays
null/RIGHT_CENSORED. Do not call this original joint-order2 production availability.

## D: PriorEvidence

Readonly prototype, default UNQUALIFIED/fail-closed. Simulation factory validates
namespace/model/scope/horizon, unique physical atoms, explicit onset/duration and finite
categorical mass. Queries bind time basis, event predicate, identity and evidence
validity horizon; deployment queries always fail. An event tail can extend past
the evidence horizon; that grants no query eligibility beyond the horizon. Joint probability is the exact
intersection under the declared categorical law (same atom p, different atoms 0),
never an inferred product. No calibration string qualifies a probability.
Existing prior provenance investigation is reused: UWB 1e-4 and IMU 1e-5 remain
unqualified experimental settings; production generator uses min(member bounds)
for intersections, not an independence product. A production revision needs an
approved physical event/time predicate and matching sensor evidence; no revised
production contract is proposed as already implemented by these simulations.

## E: scoped cost comparison

B-research = exact raw episode + adjacent support above. B-correct production
math remains the prior corrected revision; neither its old profiles nor old
removed redundant work is reconstructed to inflate a gain.

`fde_research_cost CONFIG OUTPUT normal|joint2 reference|optimized COUNT` builds
one fixed healthy seven-epoch snapshot in memory. `normal` has 98 modes/98
hypotheses; `joint2` has 98 modes/2450 hypotheses. These are **research snapshots**,
not the old joint-order2 deployed replay. Every attempt copies objects, constructs
raw maps, seals/admit the window, evaluates actual Evidence, constructs candidate
and flat jobs, executes exhaustive Flat PL, performs final validation and destroys
scoped objects/arena. Immutable input/worker startup is outside the attempt.
Attempt map indexes, source scans and seal/destruction are included. Arena, frozen
numerics and the flat scheduler are reused; mode cache and frozen memo stay off.

The producer/validator/consumer census for the two snapshots is:

| Numeric object | normal | joint2 | Producer / validator / consumer |
| --- | ---: | ---: | --- |
| Raw episode–factor responses | 294 | 294 | Real PIM/Combined FD; support/finite checks; Evidence maps |
| Frozen Gram/profile/response entries | 98 | 2450 | Existing Evidence; arena/frozen checks; PL root eligibility |
| Flat candidate roots / leaf slots | 1 / 98 | 1 / 2450 | Existing exhaustive scheduler; candidate/Gram/nullspace checks; result reduction |
| Final Gaussian PL | refused | refused | Original unmonitorable-hypothesis refusal; no retained publication proof |

The frozen KEEP fast root is ineligible when its entries include this unresolved
IMU direction; the exhaustive fallback is retained. No previously removed valid
frozen-root work is re-enabled to manufacture a reference cost.

The candidate skips three redundant integrations only for **provably absent raw
sample support**. It still does one actual integration/noise/error validation;
nonempty or sensitive cases retain all four FD integrations. Exceptions and
covariance semantics are tested. Default is OFF and production never calls it.
216 of 294 pairings are absent, reducing integrations 1176 → 528. This optimizes
research Evidence **input construction**; Evidence and Flat PL algorithms were
not changed. Their own stage times do not show a claimed gain.

Two one-attempt semantic audits compare literal numeric fields (raw maps,
Evidence/Gram/profile/rank, candidate, PL scalar/vector/terminal fields), not just
registry hashes. The transient numeric dumps are omitted from stored results;
the compact JSON records their equality/digests. Only after this freeze, four
processes ran 35 sequential attempts each (140 total). The measured median total
is normal 49.20 → 35.58 ms (27.68%), joint2 173.68 → 161.46 ms (7.04%). These are
descriptive samples; no P99, rare-event qualification or successful protected
latency claim follows. The ordinary Gaussian Flat PL in both branches **fails**
on an unmonitorable gyro hypothesis after the complete job barrier. Its original
failure is retained. Timely-availability remains false; protection latency is
null/RIGHT_CENSORED. Numerical equality and strict deadline differences are
reported separately, with all attempts and maxima preserved.

Suggested production candidates are the readonly fail-closed evidence interface,
explicit raw support/lineage diagnostics and safe malformed-proof checks after
normal review. Adopting raw episodes/correlated noise, an odometer, a bounded-range
PL or the common-reference selector in a protected production path requires the
corresponding approved fault/noise/event contract and external sensor evidence.
There is no authorized production switch in this patch.
