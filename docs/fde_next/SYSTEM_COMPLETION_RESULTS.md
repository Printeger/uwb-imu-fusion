# System completion — SC1 and SC2

Status: **MEASUREMENT_CONSTRAINED_RAW_BOUND_VERIFIED** — SC1 only.
Starting HEAD `0e88303f9f22b6c47ecb440042f3f513780caf8e`, clean tree.
FGO, FDE, raw-source-rw-v1, sigmas, risk, AL and production publication
remain the original contracts. The SC1 section below is its frozen result;
the independent SC2 result follows. SC3/SC4 remain outside this work.

W1 adds a simulation adapter certificate for the actual raw UWB batch.
Directed Taylor evaluation (the existing exp/sine kernel) encloses the exact
continuous position plus rotated lever at the rational timestamp. Directed
subtraction, square, sum and sqrt enclose ideal distance to the represented
conditional anchor. Subtracting ideal distance + sigma times the represented
source draw + represented additive fault from the actual generated range
bounds implementation residual, including timestamp/libm/Eigen and both
additions. The maximum absolute interval endpoint is rho_num. The adapter
uses draws and fault amplitudes only to certify this residue; neither is
stored or passed to the contractor. This is simulation law certification,
not online hardware fault detection. Nonfinite or excessive residues refuse.
The 1e-8 certification ceiling refuses evidence; it is not a substituted
shell epsilon. Shell delta is directed `7*sigma + rho_num` from the same
6421-source event, risk `2*6421/2^35`, with no new probability charge.

The certificate seals every measurement field and the full batch by value.
The consumer rejects correlated laws, nonregistered time/IDs, non-world frame,
subsets, altered ranges/anchors/sigma and stale snapshots. The adapter frame
is the conditional world map; UwbBatch itself has no frame field.
NONE and all eight omitted physical rows form the complete current
single-fault outer relaxation of the original 1/3-epoch episode contract.
It does not enforce cross-epoch shared amplitudes or qualify extra episodes.

W2 mathematical argument: in each omitted-row branch choose a healthy
reference. Exact subtraction of squared ranges gives
`2(a_i-a_0)'p = |a_i|²-|a_0|²+d_0²-d_i²`.
Directed intervals enclose every coefficient and RHS. Solving one coordinate
with interval remainder encloses every solution with other coordinates in
the current box; a coefficient containing zero is skipped. Intersection
therefore loses no feasible point. Each upper shell also gives
`p_j in a_ij + [-u_i,u_i]`. A squared-distance interval disjoint from the
squared shell certifies emptiness. Lower-shell sign is never selected.
All arithmetic is Boost directed interval arithmetic with `-frounding-math`
and `-ffp-contract=off`, checked by the runtime 1+2^-53 witness.
Nonfinite arithmetic falls back to the full inertial box, not an empty branch.

Bisection uses two closed boxes sharing the midpoint, whose union covers
the parent. Global budget expiry retains every queued box and every later
unvisited fault root. Hulling all retained branches gives `S subset Q subset B`.
An all-empty union reports MODEL_INCONSISTENT with infinite PL and no issuance.
Signed distance from Q to the actual center and horizontal sqrt are also
outward interval operations. By monotonicity these bounds cannot exceed the
same directed operations on B at that center. Finite overflow fallback is
explicit; it does not certify invalid input or unsupported IMU laws.

On E, the original proof places truth in B; the healthy UWB shells hold for
the true current fault projection, included among NONE/eight masks. Thus
truth belongs to S and Q regardless of FDE or the data-dependent center.
The same task event and amount apply simultaneously at all 50 epochs.
This argument does not require independent inertial and UWB sets.

W1/W2 executed checks: source certificates at 16 epochs of U0/U1/U2/U3;
100-digit independent distance oracle for all eight fault rows, zero/weak/
strong faults and both 7-sigma boundaries; 155 independently feasible lattice
points; 1D/2D/3D analytic unit-ball hulls, coplanar mirrors, near tangency,
zero height, negative shells, overflow and nonempty/zero-budget frontier.
All pass. Deterministic fixtures are geometry checks, not probability samples.

W3 integration: opt-in `SC1_RANGE_CONTRACTION=1` or
`tools/run_epoch_architecture.py raw --contract-ranges`. After the original
single native commit, inertial issue is evaluated first; SC1 validates that
same receipt, recorded actual tag/lever/time/graph, the complete original
transaction batch, sealed numeric residue and frame/law. No selected subset,
future measurements, truth, scene or injection ID enters the kernel. It does
not feed back position, velocity, rotation, bias, FGO, H/P or FDE decisions.
The paired old/new fields come from one native estimator execution. Missing
old qualification cannot be inherited; SC1 evidence refusal or model
inconsistency also refuses the opt-in served raw output. Default paths remain
unchanged. H1's existing weak injection is merely certified at generation,
with the same raw values and detector behavior as its previous runner injection.

Native fixture forces deletion of healthy 1502 at U1/15 while retaining
faulty 1501. Both this commit and the following epoch retain truth within Q.
ID/value/anchor/sigma/time/frame/law/subset/correlation/tag/lever/receipt/
box/center/expiry/IMU-law mutations refuse; changing future UWB leaves current
Q and PL exactly unchanged. Predefined development snapshots U0/U1 epoch31
show HPL 42.4638/42.4636 -> 2.96203/2.96305m and VPL
30.4284/30.4286 -> 5.63746/5.63767m. These are paired raw bounds at the same
center, not a comparison with frozen Gaussian. The final W4 acceptance follows below.
Frozen budget: 256 visited boxes globally, 8 propagation passes, no default
splitting. Optional depth is capped at16 and passes at64; budget-limited
splitting tests explicitly retain frontier. Default nine root boxes suffice.

## W4 — frozen same-stream acceptance

Candidate `2b7fe1c` was frozen for U0/U1 full50, U2/21, H1/5 and U3/I2
negatives. Subsequent input guards only refuse NaN/inverted inertial boxes or
nonfinite shell arithmetic before constructing an interval; they do not change
any accepted positive operation. Targeted mutation and arithmetic checks cover
these guards. No seed search, source/sigma/AL change, repeated positive campaign
or Gaussian-min combination was used.

| Stream | Scheduled / actual single commits | New qualified / contracted | Unchanged / fallback / refused | Correct exclusions | <=50ms / late | PE over new bound |
|---|---:|---:|---:|---:|---:|---:|
| U0 | 50 / 50 | 50 / 50 | 0 / 0 / 0 | 0 | 6 / 44 | 0 / 50 |
| U1 | 50 / 50 | 50 / 50 | 0 / 0 / 0 | 1 | 5 / 45 | 0 / 50 |
| U2 | 21 / 21 | 21 / 21 | 0 / 0 / 0 | 3 | 6 / 15 | 0 / 21 |
| H1 | 5 / 5 | 5 / 5 | 0 / 0 / 0 | 0 | 5 / 0 | 0 / 5 |
| U3 | 3 / 3 | 0 / 0 | 0 / 0 / 3 | 0 | 3 / 0 | not qualified |
| I2 | 20 / 20 | 0 / 0 | 0 / 0 / 20 | 0 | 6 / 14 | not qualified |
| missing-history | 3 / 3 | 0 / 0 | 0 / 0 / 3 | 0 | separate negative | not qualified |
| missing-IMU-evidence | 3 / 3 | 0 / 0 | 0 / 0 / 3 | 0 | separate negative | not qualified |

Acceptance flows: 155 scheduled/actual commits, 126 qualified paired outputs,
29 refused, zero healthy exclusions. Formal/protected/application outputs are
0/155. U1/15 actually removes 1501, retains7, considers all9 detector candidates;
SC1 still consumes all8 original IDs. U1/16–19 and U2/18–21 are qualified and
covered, with full task risk unchanged. H1/3 weak contamination is unalarmed,
not excluded, and H1/4 retains a nonzero historical bound. U3 retains the legacy
mirror/geometry refusal; I2's healthy-IMU source certificate is invalid. Finite
frozen Gaussian I2 numbers never gain raw qualification. Lateness changes
application/timeliness assessment, not the set or event risk.

A harness classification error originally treated H1 as a negative and aborted
at that assertion after its five actual commits. It was corrected to the
healthy-IMU positive class, and only H1 and remaining negatives were run.
Those additional five attempted rows are explicitly a failed harness attempt,
not deleted mathematical failures or added successful probability samples;
total final-phase attempted commits including that attempt =160. The original
U0/U1/U2 121 validated rows were retained in the summary and not rerun.
Development also used the prescribed two epoch31 snapshots and a 3-epoch
opt-in/off exact-equivalence prefix; these are separate fixtures, not the final
50-epoch denominator.

All 126 qualified pairs have new HPL/VPL <= the old interval bound at the
identical center. Directed output arithmetic is the same on both boxes;
observed outward-only widening is zero. The independent 100-digit tests check
old and new horizontal synthesis separately. Replay PE checks support correct
integration; the common-event proof above establishes the risk theorem.

Each statistics tuple below is mean / median / P95 / max, metres.

| Stream | Old raw HPL | New raw HPL | Old raw VPL | New raw VPL |
|---|---|---|---|---|
| U0 | 39.2374 / 32.5697 / 85.7064 / 93.7941 | 2.90575 / 2.89857 / 3.01375 / 3.03059 | 27.9136 / 23.3768 / 60.4766 / 66.0843 | 5.34872 / 5.33359 / 5.67254 / 5.80384 |
| U1 | 39.2372 / 32.5705 / 85.7062 / 93.7939 | 2.91102 / 2.89878 / 3.01311 / 3.29132 | 27.9139 / 23.3771 / 60.4766 / 66.0843 | 5.36459 / 5.34104 / 5.74341 / 6.03281 |
| U2 | 16.1558 / 15.2088 / 24.4882 / 25.8329 | 2.97132 / 2.90250 / 3.37029 / 3.65112 | 11.5144 / 10.7742 / 17.5684 / 18.5032 | 5.51483 / 5.44128 / 6.14031 / 6.80874 |

| Snapshot | Old HPL / VPL | New HPL / VPL | New/old H / V |
|---|---|---|---|
| U0/31 | 42.4638 / 30.4284 | 2.96203 / 5.63746 | .06975 / .18527 |
| U1/31 | 42.4636 / 30.4286 | 2.96305 / 5.63767 | .06978 / .18528 |
| U0/50 | 93.7941 / 66.0843 | 2.94552 / 5.21684 | .03140 / .07894 |
| U1/50 | 93.7939 / 66.0843 | 2.94534 / 5.21680 | .03140 / .07894 |

At U1/31 the old per-axis PL is (30.0008,30.0518,30.4286), new
(2.16912,2.01856,5.63767); B widths (59.9997,59.9997,59.9997) become
(3.87768,3.71585,10.16082). At U1/50 old per-axis PL is
(66.8037,65.8374,66.0843), new (2.11500,2.04983,5.21680); B widths
(131.6094,131.6094,131.6094) become (4.04608,4.01411,9.80914).
U0 has the same contracted widths at those two epochs (same healthy raw stream)
with slightly different actual-center transfer. The prescribed four snapshots
shrink by metres, far beyond directed numerical residue.

| Stream | Contractor ms mean / median / P95 / max | Full service ms mean / median / P95 / max |
|---|---|---|
| U0 | .29352 / .27496 / .37985 / .45939 | 144.911 / 166.769 / 174.280 / 186.038 |
| U1 | .29772 / .27658 / .38207 / .75166 | 145.262 / 167.591 / 175.965 / 188.895 |
| U2 | .27153 / .27869 / .31966 / .32527 | 109.884 / 151.295 / 166.405 / 167.409 |
| H1 | .27775 / .27889 / .28154 / .28211 | 21.113 / 19.876 / 34.552 / 36.369 |
| U3 | not invoked | 13.839 / 13.832 / 19.301 / 19.908 |
| I2 | not invoked | 108.366 / 139.920 / 171.277 / 185.344 |

Full service includes IMU inclusion, original H/P, A/B/FDE, verification,
unique native commit, contractor/binding, final output assessment, H update and
cleanup. H dominates: U0/U1 means 141.466/141.573ms; prediction .307/.340ms,
A .064/.072ms, B .427/.444ms, commit 1.687/1.813ms. Contractor cost is part of
finalization (.343/.350ms means), not an extra omitted fee. Source-law generation
and certification cost 47.45/47.28ms once at initialization, outside epoch
service. Full50 is a 2.5s declared task, not an arbitrary-length online guarantee.

Every qualified epoch visits9 roots; default frontier0 and queue peak1.
Retained roots: U0 always9, U1 mean8.92 (epoch15 retains five conservative branches, including the true fault projection),
U2 mean8.43. No resource fallback occurred in the final streams; forced-budget
fixtures retain unvisited fault roots and nonempty frontier. H storage peaks
3,088,408 bytes; measured maximum RSS over the full positive invocation and
its harness is 46,944KiB (includes the aborted H1 classification). Separate
H1/U3/I2 harness peaks 31,124/30,972/31,788KiB. These are observed RSS and short
service samples, not WCET or hard real-time evidence. Status: NOT_REALTIME.

AL evaluation reads the identical immutable trace and asserts no row mutation.
At AL=2/3,5/5,10/10 the numeric qualified passes are U0 0/4/50 out of50,
U1 0/4/50 out of50, U2 0/3/21 out of21, H1 0/2/5 out of5. PL, selected IDs,
state, status, H, risk and receipt remain unchanged. All application availability
remains false because the original policy/hardware qualification is unchanged.
Legacy AL=2/3 was not relaxed.

Final related safety regression passes: `raw_range_checks` (including
100-digit old/new horizontal-norm oracle, NaN/inverted-box refusal and actual
wrong-FDE native fixture), `hybrid_raw_checks` (6421-source/full15 RW/ADC/shared
cross/retired-prefix and continuous inclusion), and `hybrid_checks` (20 fixed
H/P/B outputs plus 48 native outputs, source/correlation/lifecycle/AL/receipt
and 51ms lateness downgrades). The original assertions were retained. A separate two-commit, one-epoch
nonhybrid H1/U0 compatibility fixture matches raw values, state, FDE and PL
exactly. All62 development snapshot pairs also pass100-digit endpoint-distance
checks against the actual emitted CSV tag. These fixtures are outside the
final stream denominator.
The regression ran once after the final negative-input guard changes; it did
not rerun N0–N4 campaigns or old S1–S4/HGIR studies.

| Acceptance | Evidence and conclusion |
|---|---|
| A01 | HEAD/dependencies audited; original GTSAM 4.2a5/native backend, FDE, raw-source-rw-v1 parameters, AL and release restrictions preserved. H1 nonhybrid compatibility remains its original U0 input. |
| A02 | Actual noisy shells `7*sigma+rho_num`; directed continuous-tag/source certification, both signed 7-sigma zero-distance tests; same6421 inventory/event. |
| A03 | Mandatory NONE/eight physical masks; explicit CURRENT_SINGLE_FAULT_OUTER_RELAXATION_OF_EPISODE_SCOPE; actual wrong FDE still covered. |
| A04 | Necessary slab/shell proof `S subset Q subset B`; independent100-digit and lattice/analytic/mirror/near-singular oracle. |
| A05 | Unvisited branches/frontier retained; zero/nonempty budget tests, nonfinite arithmetic fallback, all-empty refusal with infinite PL. |
| A06 | Full8 raw batch sealed against mutations, same recorded actual tag/lever/receipt/graph/time, original unique native commit. |
| A07 | U0/U1 full50 paired nonwidening; all four prescribed31/50 snapshots strictly shrink; U1 exclusion1501+four following epochs, U2 three exclusions+18–21 pass. |
| A08 | U3/I2, missing history/IMU, mutation, expired evidence, budget and51ms negatives; immutable AL2/3,5/5,10/10 postprocessing; no hardware or IMU-fault qualification. |
| A09 | Same risk3.7375139072537422e-7 over full50; no refresh/branch charge; Gaussian nonlinear raw proof remains incomplete. |
| A10 | Full denominators, failed harness attempt, cost/RSS, minimal repro and four local work-package commits; own scratch removed after inspection. |

Minimal reproduction from repository root (no production publication):

```bash
python3 tools/hybrid_raw_research.py range-check --build
python3 tools/run_epoch_architecture.py raw --contract-ranges
python3 tools/hybrid_raw_research.py noise-model
python3 tools/run_epoch_architecture.py check-hybrid --build
```

`raw --contract-ranges --scene U1` runs one estimator full50 with both bounds;
`--scene U2` preserves21 epochs; `--scene H1` uses the weak five-epoch prefix.
Missing-evidence reproduction uses the same runner with final argument
`history-invalid` or `imu-invalid`, for example:

```bash
BIN=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl
mkdir -p .cache/sc1-range-contraction
SC1_RANGE_CONTRACTION=1 HYBRID_SHARED_EPISODE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 LD_LIBRARY_PATH="$BIN/..:${LD_LIBRARY_PATH:-}" "$BIN/epoch_raim_runner" config/hybrid_raw.yaml .cache/sc1-range-contraction/negative.csv U0 hybrid on 3 history-invalid
```

Automatic stream CSVs live only in `.cache/sc1-range-contraction/` and are
removed by the CLI; local direct-run files must be removed after inspection.
Permanent artifacts are the kernel/source adapter, required checks and CLI,
and this report; no box dumps, raw data, binaries, backups or plot campaign.
Local stages: W1 `15d5b37`, W2 `948c221`, W3 `2b7fe1c`, W4 the commit titled
`test(sc1): close paired raw range contraction acceptance`.

**SC1 conclusion: MEASUREMENT_CONSTRAINED_RAW_BOUND_VERIFIED.**
This establishes only the declared healthy-IMU, conditional simulation,
50-epoch/no-carry-in/single-UWB-episode scope. It does not establish SC2–SC4,
the original Gaussian PL's raw nonlinear theorem, complete system readiness,
hardware qualification, application admission or real-time completion.

## SC2 — IMU fault scope and legal recovery

Starting local HEAD `50b7582`, clean tree, SC1 retained. W1–W3 below describe
the staged implementation; W4 below is the final scoped acceptance. Legacy I2 (`config/fde_next.yaml`) uses accel-x +20m/s² on raw
sample times [.705,1.205), without the matched initial Gaussian/RW generator.
Matched `config/hybrid_raw.yaml` has actual initial15D observation and
sample-grid RW. Their numerical results are distinct laws, not an improvement
ratio. The old healthy source certificate correctly fails on injected IMU;
that failure is not the new online detector. Old `fullStateBridge` has zero
bias drift under its stated constant-bias premise and cannot prove the RW case.

W1 preregisters `config/sc2_imu.yaml`: nominal raw-source-rw-v1 unchanged,
IMU-only single constant episode, six axes, onsets1–50, durations50/500ms,
unbounded amplitude, no initial carry-in, full50 event, all UWB healthy.
Representative accel-x +20m/s² and gyro-x +1rad/s start at epoch15; short
accel-x +1m/s²/50ms is a weak negative. Final recovery stream is30 epochs,
covering the shared-tail bridge at25, restored IMU26 and following26–30.
Independent world acceleration <=.04m/s² and angular speed <=.2rad/s are
registered motion assumptions of the original smooth-motion law; they are not
extra sensors or optimizer weights. The original initialization is the only
fault-independent boundary, maximum task bridge2.5s. All601 explanations
(NONE plus600 physical episodes) must fit; capacity failure refuses the scope.

`Sc2ImuMonitor` consumes only observed raw samples, initial observation and
registered source/domain parameters. In E a healthy sample's bias departure
is bounded coordinatewise by the original initial-bias error, the sum of all
preceding RW increments and ADC error. Its ideal body force coordinate lies
within +/- (g+A); gyro coordinate within +/-Omega. Directed arithmetic derives
an interval H_j for each healthy coordinate. A value outside H_j proves NONE
inconsistent; on E this detector has no healthy false alarm, hence its full-task
conditional false-alarm upper is at most the existing E complement. It makes
no arbitrary-amplitude detection-power claim. For each episode, outside-support
samples must satisfy H, while inside-support fault coordinates intersect one
shared amplitude interval `[m_j-H_j.upper,m_j-H_j.lower]`. Only proved-empty
explanations are removed. This preserves the true episode/parameter on E even
when weak faults are not alarmed. No draw, truth or injection label is an input.

Support closure uses integer half-open sample times, maps every raw point to
both adjacent trapezoids/PIMs at shared boundaries, and marks all later
state/biasHat/reference dependencies. For [.705,1.205):100 raw points, direct
PIMs15–25, downstream dependencies through the current epoch. For50ms:
10 points and PIMs15–16. W1 checks pass for all six axes, actual raw alarms,
shared-theta retention, nominal30 prefix, capacity/prefix refusal, and an
independent literal-sample perturbation/reintegration response oracle at the
matched-RW two-PIM shared endpoint. These are finite deterministic checks,
not an empirical tail certificate. Nested research-law loading was made
recursive so SC2 inherits the actual SC1 nominal law instead of copying it.

W2 implements `Sc2IndependentBridge`, a new fault-independent full-state proof,
not a relabeling of healthy-IMU inclusion. With original observed boundary O,
`r_p=||7 sigma_translation||+rho_initial`, `r_vj=7 sigma_vj+rho_vj`, and
time t=k/20, true body position belongs to
`O.p + [-r_p,r_p] + t*(O.v +/- r_v) +/- A*t²/2`, velocity to
`O.v +/- r_v +/- A*t`. Initial attitude error plus `Omega*t` bounds matrix
operator distance by `min(2,||7 sigma_rotation||+rho_rotation+Omega*t)`.
Add represented initial R times lever plus the diameter times lever norm for
a tag box. Matrix-entry envelopes preserve all possible R, without pretending
their independent entries define a unique rotation. Each bias coordinate keeps
its original initial-observation error plus the SAME first10k RW increments,
`7*(sigma_bias+10k*sigma_RW*sqrt(1/200))`, and directed initial/process
roundoff. The adapter now separately bounds accumulated RW arithmetic residue
against its exact source mapping; that term is consumed by the new bias box.
No zero-bias premise, polluted posterior, range-derived velocity measurement,
Gaussian covariance or optimizer regularization enters these equations.

The motion bounds are checked against directed
`norm(abs(amplitude)*frequency²)` and registered absolute angular rate before
use; reducing A below the actual declared analytic domain refuses the bridge.
The valid boundary is timestamp0 original observation only, with no carry-in.
All branches share this conservative full-state envelope, which contains the
true branch independently of the selected action or any IMU corruption within
the declared task. Current all-healthy UWB shells safely contract only its tag
projection; no contracted tag box is written into p/v/R/bias. SC1's consumer
continues to require all9 UWB masks and explicitly rejects the new healthy-only
kernel option; SC2's separate contract uses only NONE (all8 ranges healthy).
SC2 does not claim simultaneous UWB+IMU protection.

On E the initial envelope holds; integrating independent acceleration/angular
bounds and the true RW retains the full state, so the true tag belongs to B_k
for every physical IMU hypothesis. The monitor preserves the true h/theta via
necessary raw intervals. Intersecting the healthy range shells preserves the
true tag; directed contraction gives Q_k and signed transfer to the actual
committed tag. All candidate/center choices are covered simultaneously by
these inclusions. Risk remains `2*6421/2^35=3.7375139072537422e-7` over full50,
without new source draws, per-branch charge or per-epoch refresh. This is a
conditional sup-fault task bound, not hardware HMI qualification.

Fault forward-map certification subtracts the additive raw fault inside the
directed source evaluator, including rounding of that addition. Its evidence
contains numeric residue only, never amplitude/onset/axis labels or actual
draws. The old healthy certificate remains invalid on SC2_A/G/S injected input;
this failure is not used to alarm. The online monitor still consumes actual
observations and registered physical intervals. Default SC1 generation avoids
the extra fault-source certification pass.

W2 checks: new source cert valid while healthy cert invalid; full p/v/R/bias/tag
envelopes contain matched-law witnesses at15/25/26/30 for accel, gyro and weak
short inputs; 100-digit extremal integral/RW endpoint oracle; expiry and invalid
physical domain refusal. Actual FDE-off native commits at15/16 retain polluted
centers but issue the new same-event raw bound correctly. Prefix/receipt/frame
mutation, polluted boundary and unrepresented simultaneous two-axis raw
violations refuse. The unaffected SC1 range checks also pass. Native recovery
and final stage acceptance remain pending W3/W4.

W3 adds the opt-in `SC2_IMU_RECOVERY=1` coordinator to the existing native
runner and CLI (`check-sc2`, `sc2`). The frozen raw candidate bank determines
a conservative monotone quarantine interval. An alarm removes each selected
active CombinedIMU group from the earliest still-plausible dependency onward,
and adds exactly one native pose/bias bridge pair per removed historical group
plus the complete current pair. Missing original x0 or raw recoverability
refuses before commit; contaminated marginal priors cannot substitute. The
existing retained-original-prefix routine performs exactly two finite GN
rebuilds (bounded iteration counts); only its candidate initial guess is
passed into the pending transaction. Receipt/ledger checks prove the actual
historical deletions and exactly one backend update. There is no second
estimator and no post-commit state write. The raw bound comes from W2, not the
candidate optimizer covariance.

The support-union quarantine persists through epoch25, including the shared
1.200s endpoint. Epoch26 returns to the actual CombinedIMU factor using a new
integration of its complete healthy raw slice at the ORIGINAL observed bias
reference; true process bias RW remains in both physical law and protection
envelope. Restoration is derived from the registered retained support interval,
not the injection-stop label. Native tests check actual slots and original
biasHat, missing history/x0 refusal, same-data shadow candidate identity and
coverage through this first restored commit. Six-axis zero-amplitude/cancelled
explanations remain present without manufacturing an alarm.

Development snapshots (not final acceptance denominators): accel-x and gyro-x,
16 commits each, all qualified. Both first raw alarm at0.705s and first native
isolation at epoch15 (batch ends0.750s). Accel's possible onset union initially
starts at epoch6: 9 historical PIMs plus the current PIM are replaced; gyro
initially starts at15. At15, accel H/V=1.46192/2.67915m, gyro=1.46226/2.67522m.
The retained-graph diagnostic candidate displacement is about0.772/0.783m,
which EXCEEDS the old0.25m local position domain; old diagnostic status remains
`PIM_HISTORY_UNIFORM_BOUND_MISSING`. That Gaussian local gate was neither
widened nor certified. Qualification instead uses the new global independent
physical source theorem covering arbitrary committed centers, full original
initial uncertainty and true RW. No claim of accurately recovered full-nav
covariance is made. Frozen final streams and acceptance remain pending W4.


### W4 frozen native acceptance and measured limitations

Status: **IMU_FAULT_SCOPE_VERIFIED**, with `verified_scope` split into the
conditional mathematical six-axis domain and actually exercised recovery
subdomain in the handoff below. Accel-x/500ms and representative gyro-x/500ms
both complete the native automatic isolation and IMU-return chain. Other
axis/onset/amplitude combinations have a uniform conditional protection
argument and six-axis response checks, but their native recovery performance
is OPEN, not inferred from this one gyro direction. SC3, SC4, joint faults,
application AL and deployment are not completed.

Frozen functional source SHA: `32360ecfd7514f7a7beb6bb296470643513c93b3`.
The W4 commit adds only direct negative/regression tests and this conclusion;
no functional mathematics or preregistered witness changed after freezing.
One invocation of `python3 tools/run_epoch_architecture.py sc2` runs six scored
30-epoch streams, one same-input redacted30 stream and SC1 U1/19 compatibility:
229 scheduled/attempted/committed, 0 aborted/cancelled/unattempted, 229 finite
and raw-qualified, 229 contracted, 0 fallback/refused; 0 scored PE exceedances
among199 scored outputs, with30 redacted outputs deliberately unscored.
All210 SC2 commits retain the complete8-row native UWB batch, exactly one
backend update and the constant full-task risk3.7375139072537422e-7.
SC1 U1 retains its own9 physical range branches, actually excludes1501 at15
and contracts all19; the four follow-up commits16–19 are qualified. Its full
legacy H/P branch remains costly; a directed19-row metric audit records13/19
late at the50ms service criterion,
whereas the six timed SC2 streams have0/180 late. Redacted timing is not used
as a performance comparison (truth scoring is disabled).

The strong A/G witnesses start at sensor timestamp0.705s. Their first affected
batch ends0.750s, raw alarm first sample0.705s, isolation commit at epoch15;
that is45ms of acquisition to the first available batch, plus measured service,
not a wall-clock hardware TTA/queue guarantee. Both bridge epochs15–25
(11 commits, each exactly two rebuilds), and actually rejoin CombinedIMU at26
with five consecutive restored propagation commits26–30. The injection stops
at1.205s; PIM25 remains suspect due to the1.200s shared raw endpoint. At25
the surviving physical episode bank is one, but no true label was read.
The delayed17 witness retains the polluted native15/16 commits, then purges
history from epoch8 through16 at17, bridges17–25 and rejoins26–30. Its first
raw alarm remains0.705s; isolation at0.850s was deliberately delayed by a
registered test control, rather than credited as automatic prompt response.
The weak+1m/s²/50ms witness never alarms: NONE remains possible, all30 actual
commits are still protected by the independent full-state proof. Thus this
negative is not rejected by its scene name or the failed healthy certificate.

Same actual30-epoch input with scene output `REDACTED` and all truth reads for
scoring disabled yields exactly identical candidates, actions, graph IDs,
receipts, state, bound coordinates, H/V and task risk at all30 commits.
Independent same-input shadow candidate checks also agree, with no label
argument in monitor/coordinator/bridge interfaces. Offline label/scoring and
source-adapter numerical validation do not participate in FD/action selection.

Each row in this paired table is30 actual/scored/contracted commits. Values
are mean / median / P95 / max; PE and PL in m, velocity error in m/s, rotation
error in rad. FDE-off/on use identical physical draws and input samples, with
bit-identical committed tag/v/quaternion for the first14 healthy epochs.

Tag output and error

| Stream | hpl | vpl | horizontal_pe | vertical_pe |
|---|---|---|---|---|
| SC2_A-off | 1.5662568/1.5562569/1.7230799/1.8110788 | 2.9342508/2.8595102/3.2984265/3.3042523 | 0.11147395/0.07121812/0.26211135/0.27382342 | 0.17227005/0.05105432/0.57902726/0.58576838 |
| SC2_A-on | 1.5294371/1.5253252/1.6530924/1.6875554 | 2.8916332/2.8588933/3.1918016/3.2497189 | 0.064178254/0.059905361/0.14218363/0.15466215 | 0.055308198/0.038904028/0.12321773/0.13432393 |
| SC2_G-off | 1.5422968/1.529139/1.6963915/1.7484034 | 2.9007281/2.8588933/3.2159218/3.3234106 | 0.062614031/0.059905361/0.10322719/0.14450141 | 0.047316182/0.036607578/0.10085514/0.12020374 |
| SC2_G-on | 1.5277415/1.5253474/1.6616319/1.6875554 | 2.8927099/2.8588933/3.1924341/3.2857979 | 0.064167804/0.054880992/0.15331177/0.19834765 | 0.061500553/0.045467856/0.13231237/0.18234861 |
| SC2_S-weak-negative | 1.538758/1.5395883/1.6685817/1.6875554 | 2.8969035/2.8588933/3.2028091/3.3004479 | 0.049958126/0.048278245/0.07937195/0.089212407 | 0.038997772/0.035187582/0.091052049/0.12020374 |
| SC2_A-delayed17 | 1.5317657/1.536959/1.6487513/1.6875554 | 2.8904733/2.8555888/3.1923412/3.2453707 | 0.053336599/0.047226128/0.093720607/0.15473437 | 0.046774366/0.03458169/0.12326396/0.1344196 |

Full navigation estimate diagnostics

| Stream | velocity_error | rotation_error |
|---|---|---|
| SC2_A-off | 1.7443714/1.2213753/4.141185/4.5256405 | 0.33334223/0.10895265/0.89302214/0.90992108 |
| SC2_A-on | 0.55549349/0.39216213/1.5434941/1.8996587 | 0.19024039/0.11222941/0.58101868/1.1999434 |
| SC2_G-off | 0.53961969/0.46292879/0.97810485/1.1326836 | 0.20826064/0.14317519/0.42535787/0.45254264 |
| SC2_G-on | 0.62507551/0.29583371/2.3118088/3.5769127 | 0.18828773/0.07484959/0.93552831/1.7808487 |
| SC2_S-weak-negative | 0.31851273/0.25674013/0.57020908/0.86835852 | 0.069277835/0.069358431/0.093334214/0.094253709 |
| SC2_A-delayed17 | 0.54504981/0.4187837/1.2396107/1.574392 | 0.15249708/0.10737995/0.35331408/0.66423548 |

Service time (ms)

| Stream | candidate_ms | range_contraction_ms | full_ms |
|---|---|---|---|
| SC2_A-off | 0.0010917667/0.0009525/0.002222/0.002519 | 0.0454638/0.0440675/0.05291975/0.065061 | 2.1791011/2.146334/3.9801468/6.310948 |
| SC2_A-on | 2.6548523/0.034498/9.3083219/11.76623 | 0.055002667/0.0489225/0.09957375/0.15441 | 4.5781565/2.5336995/12.297196/14.589193 |
| SC2_G-off | 0.0010882667/0.000988/0.0020201/0.00213 | 0.047387367/0.044714/0.051211/0.106614 | 1.9795354/1.8863485/3.7553712/4.601319 |
| SC2_G-on | 3.9844286/0.0354505/13.587577/14.172504 | 0.0517307/0.049512/0.0726027/0.111438 | 5.8097596/2.41574/16.239895/17.187232 |
| SC2_S-weak-negative | 0.0019023/0.000825/0.0028694/0.028519 | 0.0478589/0.0446315/0.0658682/0.067643 | 1.8650802/1.8601955/2.979595/3.130938 |
| SC2_A-delayed17 | 2.6244556/0.001327/10.069715/11.769709 | 0.0485511/0.047636/0.05762105/0.063903 | 4.5471784/1.901378/12.432784/14.019093 |

The service boundary includes ingest/monitor, native preparation, candidate
rebuild, prefix maintenance, unique commit, final state read, receipt checks,
raw full-state bridge, shell contraction, transfer and offline scoring; it
ends before CSV serialization/file I/O. Source initialization is separate:
56.05–59.34ms per SC2 stream (both numeric certifications and scenario build);
SC1 compatibility source init20.7ms. Prefix retained-group peaks are63(off),
83(accel-on),74(gyro-on),63(weak),81(delayed). The physical hypothesis bank has
600 entries plus NONE; the maximum range box queue is bounded256, passes8,
no splitting, no omitted frontier. This is finite measured service on this
PC, not WCET, acquisition timing, sustained ROS performance or SC4 success.
The SC2 bound consumes no legacy Gaussian H/P computation: those diagnostics
are unnecessary for this global physical proof. The unchanged SC1 chain still
runs its original H/P and remains NOT_REALTIME. These timings are not evidence
that the original combined system meets40/50ms.

Post-return evidence is retained explicitly below. For all shown epochs the
actual current IMU group is present, `imu_rejoined=1`, update count1, raw
qualified1; no raw tag contraction is fed back to v/R/bias.

| Stream | Epoch | HPL/VPL m | PE H/V m | Velocity error m/s | Rotation error rad |
|---|---|---|---|---|---|
| SC2_A-on | 26 | 1.569117/3.187419 | 0.018479/0.040099 | 1.143992 | 0.166207 |
| SC2_A-on | 27 | 1.554843/2.950810 | 0.140890/0.061456 | 1.754564 | 0.501680 |
| SC2_A-on | 28 | 1.669381/2.914401 | 0.143242/0.120105 | 1.073592 | 1.199943 |
| SC2_A-on | 29 | 1.633184/3.249719 | 0.139762/0.015781 | 1.899659 | 0.601483 |
| SC2_A-on | 30 | 1.622814/2.764989 | 0.120193/0.026658 | 1.285520 | 0.556006 |
| SC2_G-on | 26 | 1.570279/3.188825 | 0.020877/0.041505 | 1.086479 | 0.065255 |
| SC2_G-on | 27 | 1.559667/2.937978 | 0.147653/0.048625 | 1.702258 | 0.279373 |
| SC2_G-on | 28 | 1.678529/2.976645 | 0.151688/0.182349 | 1.796893 | 1.780849 |
| SC2_G-on | 29 | 1.640979/3.285798 | 0.198348/0.051860 | 2.733104 | 0.623585 |
| SC2_G-on | 30 | 1.548627/2.715473 | 0.044127/0.130331 | 3.576913 | 1.190755 |

The restored v/R estimates are NOT uniformly improved by isolation: gyro-on
max velocity error3.5769m/s versus off1.1327, max rotation1.78085rad versus
off0.45254. This is a real limitation of the retained native regularized
candidate/observable navigation accuracy. The full-state TRUE uncertainty
envelope and future tag coverage remain fault independent over the registered
finite task; it does not assert that the estimate lies near truth in every
state, or that a low-residual bridge proves truth. No Gaussian covariance,
zero-bias box or enlarged local trust gate is credited. SC3 must preserve the
same independent physical bridge or supply a new proof before relying on v/R.

AL2/3m is independent post-processing: numeric passes A-off20/30, A-on23/30,
G-off23/30, G-on23/30, weak24/30, delayed23/30. AL5/5 and10/10 each pass30/30
in all six scored SC2 streams. These copies leave state, FDE, PL, receipt and
risk exactly unchanged. All rows keep formal eligibility, publication
protection and application availability0. No policy was relaxed to obtain a
pass; production AL and risk configuration are unchanged.

Development/failure accounting is separate from the frozen acceptance:
two16-epoch diagnostic snapshot streams completed32/32 commits; W1/W2/W3
parameterized tests passed after their local changes. Two initial compile
attempts failed (W2 two-word cast syntax; W3 missing Symbol include/wrong
existing enum), and one pre-freeze Python syntax check failed (f-string quotes).
They produced no native epoch attempts; all were corrected before freezing.
The frozen stream harness had0 failures,0 cancellations; deliberate invalid
fixtures (capacity600, stale prefix/receipt, fake invalid source evidence,
wrong frame, polluted boundary, absent x0/history, expiry51, simultaneous
unrepresented two-axis violations) are expected safe refusals, not discarded
positive attempts. One final regression harness invocation omitted the existing required YAML
argument to `epoch_raim_checks`, aborting before any native epoch; its
remaining fused target was then unattempted. Both targets were run once with
`config/fde_next.yaml` and passed; the four already passed targets were not
repeated. A directed SC1 metric audit (the initial compatibility logger had
not printed lateness) adds19 scheduled/attempted/committed/finite/qualified/
contracted outputs,0 aborted/refused/fallback,13 late; service mean/median/P95/
max=104.472/128.290/166.502/168.159ms. This is separately accounted, not hidden
in the frozen229-output denominator. The unrelated earlier SC1 failure denominators remain in
the unchanged SC1 section. No long campaign/seed search was run.

### SC2 → SC3 capability handoff

| Dimension | Actual upstream capability and limits |
|---|---|
| Source / scope / method | Frozen source `32360ecfd7514f7a7beb6bb296470643513c93b3`; `sc2-single-imu-episode-rw-v1`; `raw-imu-fault-independent-bridge-uwb-v1`. W4 changes tests/reporting only. |
| Requested physical family | NONE or one deterministic constant additive raw single-axis episode: accel xyz or gyro xyz, theta in all real numbers, onset `(j-1)*50ms+5ms`, j=1…50, duration50 or500ms; half-open support, no initial carry-in. End may extend beyond task; no post-horizon claim. All8 UWB and original initialization satisfy nominal source law. |
| Verified mathematical domain | All six axes and theta values in this family, arbitrary adaptive candidate/action/committed center, issued epochs1…50 under original initialization and independently declared motion bounds. True branch/theta remains in the complete bank on E; all possible branches share one valid full-state physical outer envelope. Valid proof does not require an alarm. |
| Verified native recovery domain | Actual matched-law accel-x+20m/s²/500ms and gyro-x+1rad/s/500ms at0.705s, isolation15, bridge15–25, rejoin26–30; delayed accel isolation17 also tested. Six-axis raw response/detection/support mechanism is implemented; native recovery quality for untested directions, amplitudes, onsets and other motions is OPEN. Do not infer joint gyro capability from gyro-x alone. |
| Detection levels | FUNCTIONAL_ALARM for strong A/G witnesses; RAW_DETECTOR_BOUND for healthy false alarm <=same full-task E complement. No arbitrary amplitude missed-detection bound. A sufficient gyro raw condition is an additive shift exceeding twice the healthy coordinate radius (both signs); weak/zero/cancelled explanations can remain possible. Gaussian frozen-profile tools remain diagnostics in the original path, not this proof's power certificate. |
| Supports / history | 500ms witness100 unique raw sample indices141…240 (measurement IDs142…241), directly PIM15…25 including1.200s shared endpoint; all subsequent reference/biasHat dependencies. Alarm action quarantines union of possible supports; earliest actual A union6, G15, delayed A8. RawPrefixHistory and original observed raw prefix reused. Never silently eliminate historical fault parameters. |
| Legal boundary | Immutable original observed p/v/R/ba/bg at timestamp0, source-aware independent initial15D law. No polluted posterior, fitted range velocity or marginalized Gaussian boundary substitutes. Actual recovery requires original active x0 and recoverable native raw history; missing/marginalized prefix safely aborts before commit. Late first recovery after fixed-lag loss is not a promised success. |
| Bridge lifetime / physics | Task envelope2.5s max, world acceleration norm<=0.04m/s², angular speed<=0.2rad/s, consistent with original analytic motion amplitudes/frequencies/rate. This is a registered conditional physical domain, not hardware prior calibration. The witness current quarantine lasts11 batches0.55s; first A historical replacement span6–25 is1.0s. No arbitrary-duration bridge. |
| Full state / future tag | Independent integral p/v bounds; rotation operator diameter and matrix-entry enclosure; original initial bias plus all preceding actual RW sources and directed arithmetic residue. Cartesian outer enclosure conservatively retains correlations. UWB only contracts tag p+R*l; no contraction/reset of v/R/bias. Thus future issued tag remains valid even with substantial navigation estimation error. |
| Candidate / native action | Existing two bounded GN rebuilds of retained raw graph, original bias reference; finite result is only a pending initial guess. Native deletes each affected selected historical CombinedIMU group and adds its complete pose+bias pair, current IMU xor current complete pair. Ledger and receipt attest actual removal and one backend update. No second estimator, post-commit write or optimizer Hessian qualification. |
| Restoration | Monotone retained support interval ends by contract/evidence; last shared point remains quarantined. From26, reintegrate each healthy complete raw slice at original observed bias and commit its native CombinedIMU. This avoids inherited contaminated biasHat; physical process bias RW is still included. |
| Risk / posterior action | SAME6421 source coordinates and50epoch event E (each original standard-normal coordinate <=7); `2*6421/2^35=3.7375139072537422e-7`. Sup over registered faults/actions of any issued H/V exceedance is bounded by E complement, simultaneously all task outputs. No per-candidate, per-epoch or task renewal charge. Conditional source/physics theorem, not empirical zero-failure inference or hardware HMI. |
| Capacity / fallback / refusal | Exactly601 explanations required; capacity600 refuses whole scope. Range queue<=256/pass8, unresolved frontier retained; numeric contraction failure can retain valid physical tag box, never zero PL. Invalid boundary/source/frame/prefix/receipt, scope contradiction, time51 and missing history refuse; runner discards illegal action before commit. An unrepresented fault may mimic the declared family: no universal out-of-scope detector is claimed. |
| Missing scope | Simultaneous UWB+IMU faults, multiple axes/episodes, arbitrary onset/duration/shape/carry-in, task beyond2.5s/risk renewal, unsupported physical motions, independent hardware priors, original Gaussian PL's global raw proof, uniformly accurate navigation recovery, real-time service/WCET and application admission. |
| Normal research configuration | `config/sc2_imu.yaml` inherits `config/hybrid_raw.yaml`; no production risk/AL/source sigmas or native FGO weights changed. Explicit opt-in `SC2_IMU_RECOVERY=1`; default legacy/SC1 chain unchanged. Source adapter numerical evidence carries no actual fault labels/draws to the online classes. |
| Callable evidence | `Sc2ImuMonitor`, `sc2SupportClosure`, `Sc2IndependentBridge::envelope/issue`, `Sc2RecoveryCoordinator::prepare/committed/issue`, `sc2CommitPlan`; same existing EpochTransaction/ledger/CommitReceipt. Finite/qualified/raw source/scope fields, alarm, support bank, history purge, rebuild count, restored status, actual state/receipt/time and fixed risk are recorded in the short trace. |

Minimal reproduction (from this repository; scripts constrain thread counts,
set the native library path and delete their own transient traces):

```bash
python3 tools/run_epoch_architecture.py check-sc2 --build
python3 tools/run_epoch_architecture.py sc2
```

The finite safety suite is `sc2_imu_checks`, `raw_range_checks`,
`hybrid_raw_checks`, `hybrid_checks`, plus existing `epoch_raim_checks
config/fde_next.yaml` and `fused_epoch_checks config/fde_next.yaml`.
The final build runs these directly, not a new campaign. Target-specific
failure/repair and final test outcomes are stated below.

### SC2 acceptance against SC2-spec-1.0

| Clause | Concrete verification |
|---|---|
| A01 | Local SC1 `50b7582` preserved; original I2/legacy source unchanged; independent config/scope inherits actual matched initial/RW law. No old/new-law improvement ratio or production change. |
| A02 | Monitor/coordinator/bridge accept observed inputs only; no generator-health failure alarm. Redacted30 and no-label shadow are exactly invariant; weak scene remains qualified without alarm. |
| A03 | Six independent literal-axis perturbation/reintegration oracles, continuous shared theta, raw-to-adjacent PIM and downstream reference closure; actual active-slot removals and original-biasHat rejoin. |
| A04 | Actual sensor alarm/time reported separately from same-event false-alarm bound; no arbitrary-amplitude power or hardware TTA claim; no frozen velocity or reduced threshold. |
| A05 | New operational pathB global physical full-state bridge and healthy noisy shells. Old healthy-IMU certificate stays invalid on fault data and does not qualify this branch. |
| A06 | Original initial p/v/R/ba/bg, true nonzero RW plus source roundoff, independent integral/matrix/lever envelope; 100-digit extremal endpoint oracle. No zero-bias/CV Hessian proof or tag feedback. |
| A07 | Frozen accel30: native alarm/isolate15, complete history replacement, bridge through shared-tail25, actual restored IMU26–30, one commit and qualified conditional tag per epoch. |
| A08 | Actual gyro-x30 off/on and six-axis response family; raw alarm and two-rebuild native bridge15–25, actual restoration26–30. Other direction recovery is explicitly OPEN; no joint claim. |
| A09 | Weak/cancelled explanations retained and protected; actual deliberately wrong healthy-IMU deletion fixture remains covered; stopping support and delayed history purge verified; invalid source, stale receipt, missing history/x0, capacity and51 expiry safely refuse. Unrepresented two-axis violation inconsistent, without a universal domain detector claim. |
| A10 | One common event and sup-fault/adaptive-action theorem, immutable full batch, world lever tag from actual one-commit state and version/time receipt; numerical source certification separated from hardware admission. |
| A11 | Full finite stream denominators, development/harness failures and costs below; original U1 compatibility, raw source/correlation/native/core regressions; immutable AL post-processing with all application/formal flags0. |
| A12 | Capability table, normal minimal config/API/commands, verified native subdomain and OPEN limits; four locally committed work packages, no push; only source/config/tests/CLI and this unique report retained. |

SC2 status is **IMU_FAULT_SCOPE_VERIFIED** for the stated conditional family and
actual recovery subdomain; it is not SC3/SC4, `JOINT_TARGET_COMPLETE`, hardware
qualification, uniformly accurate full navigation or whole-system completion.
`APPLICATION_AL_UNQUALIFIED`, `DEPLOYMENT_UNQUALIFIED` and
`SC4_PERFORMANCE_OPEN` remain. Any SC3 combination must establish its own joint
physical hypotheses and raw inclusion; intersecting the two independent SC1
and SC2 scopes is not a valid joint theorem.


Final regression outcome: all six distinct targets PASS. Initial batch:
6 scheduled,5 attempted,4 passed,1 harness-aborted before epochs,1 unattempted;
corrected existing-CLI follow-up:2/2 passed. The final additional wrong-healthy-
IMU action/stale-invalid evidence fixture reruns only `sc2_imu_checks`:1/1 PASS.
Thus8 target invocations attempted,7 passed,1 corrected harness abort, with no
unresolved failure. The raw source oracle checks all6421 unique source IDs,
full15D actual RW/ADC/history/cross-covariance (`max_cov_relative=6.12865e-11`),
and continuous inclusion; SC1 range geometry/tamper/frontier/wrong-UWB-action
and explicit rejection of healthy-only range masks PASS. Existing native
hybrid correlation/lifecycle and UWB WLS/Schur/lever/recovery checks PASS.
The new wrong-IMU fixture actually replaces a HEALTHY current IMU at14 with
its native complete bridge pair and still protects the actual tag, then
protects polluted FDE-off15/16; it is not only a hypothetical selection claim.

Local package commits: W1 `e63fcba`, W2 `136665a`, W3 `32360ec`, and this W4
acceptance/test commit (obtain its exact hash with `git log -1`). Nothing was
pushed. Own `.cache/sc2-imu/` CSV/log/build diagnostics and generated Python
bytecode are cleaned after aggregation. No new figure, evidence database,
backup, binary or duplicate results document is retained. Existing SC1
artifacts and user files are preserved.

## SC3 — unified physical faults and joint fusion

Starting local HEAD `c254e46`, clean tree. SC1 and SC2 are preserved. This stage
uses SC3-spec-1.0 and the actual SC2 handoff above; joint qualification is not
inherited from either single-sensor proof. Production source law, weights,
risks, AL and release policy remain unchanged.

W1 preregisters `config/sc3_joint.yaml` on the same50epoch/6421-source law,
original initial15D observation, actual ADC/RW and lever/anchors. Requested:
H0, one UWB single-anchor constant episode (onsets1–50, durations1/3epochs,
unbounded metres), one IMU single-axis constant raw episode (six axes,
onsets1–50, durations50/500ms, unbounded physical amplitude), and BOTH allowed
families UWB+accel and UWB+gyro, overlapping or staggered. No carry-in or task
renewal; same independently declared world acceleration .04m/s² and angular
speed .2rad/s. Neither category can disappear following local FD PASS.
Frozen intended witnesses are original+2.25m UWB anchor0/3epochs at15 and
SC2 accel-x+20m/s² or gyro-x+1rad/s/500ms at0.705s; staggered UWB begins12.
Native recovery in other directions/onsets remains unverified, not inferred.

Implemented at W1: `HypothesisGenerator::generateRawJointResearch` uses the
existing FaultUnit/FaultModeBasis/FaultHypothesisV2 and pair-family rule to
represent an exact factored Cartesian product: 800 UWB episodes,300 accel,
300 gyro,240000 UWB+accel,240000 UWB+gyro plus H0, total481401 physical
hypotheses. A bijective indexed recipe keeps mode storage1400 and distinct
physical source/time/constant-parameter identities; no dense481401-row registry
or mode copies. The initial logical evaluation counter is0: generation is not proof
consumption. W2 will count recipe-covered branches only after consuming all9 current
UWB projections and the independent full-state envelope at W2. It must retain
all physical branches when numerical roots are shared. Capacity1399, duplicate
identity and missing consumption refuse, not truncate. Action cap18 is
preregistered separately from physical identity counts; range queue256.

Verified at entry: SC1 UWB-only theorem and SC2 IMU-only conditional theorem
remain their own scopes; NO JOINT RAW/NATIVE QUALIFICATION is claimed at W1.
The reused SC2 initial full-state physical envelope is independent of UWB,
but its all-healthy-shell contraction is not a joint proof. W2 must replace
those shell premises before any joint output can qualify. W3 must jointly
remove affected UWB and complete raw/PIM/history dependencies in one native
transaction; W4 must establish actual UA/UG and post-stop chains.

W1 actual parameterized check PASS: all481401 indexed identities traversed,
category counts exactly match, two-mode hypotheses retain independent blocks,
SC2 raw support mapping reused, duplicate/missing-mode/capacity1399 refusal.
This is a registry/support verification, not a joint source theorem or native
recovery. No stream/epoch qualification was claimed or tested in W1.

W2 supplies a NEW joint proof consumer, `Sc3IndependentJointBound`. It uses
only SC2's ORIGINAL-initial independent physical FULL-STATE envelope B_k,
which never depends on healthy UWB or measured IMU. For a physical pair h,
form `X_h = physical_X intersect (healthy rows' noisy UWB shells)`; the true
joint branch belongs on the SAME E. Its current UWB projection is NONE or
one of8 anchor omissions. Relaxing past/future shared-theta constraints may
only enlarge X_h. Each physical episode branch is therefore contained in one
of the9 current shell intersections with B_k; their UNION, NOT a healthy-UWB
intersection and not two sensor PLs, gives Q_k. IMU-dependent state/bias
correlations remain conservatively enclosed in the independent full-state
Cartesian envelope. Shell intersection changes only current tag projection.
The consumer binds the entire481401-branch recipe only after a valid9-mask
kernel result, original raw-prefix identity, full sealed8-row batch and
source/time/frame/domain checks. A final issue also verifies actual single-
commit receipt/version/time and signed transfer to actual world lever tag.

For every registered joint deterministic h/theta and every data-dependent
candidate/action/center, on E true tag belongs to Q_k. Thus
`sup_h,theta P(exists issued k<=50: PE_H>HPL or PE_V>VPL)` is bounded by
`P(E^c)<=2*6421/2^35=3.7375139072537422e-7`. No new stochastic source, hardware
fault prior, independent prior product or per-action risk charge is introduced.
All mode/pair identity and sup-fault semantics survive the9-root projection
relaxation, including local PASS and missed/incorrect isolation. This does
not certify out-of-scope physical motions, initialization, horizon or hardware.

Two actual16-epoch FDE-off fixtures (32 commits total, all finite/qualified)
use joint raw input with original+2.25m UWB and original accel/gyro episodes.
At15, A H/V=3.07720/6.03422m, G=3.07152/6.03461m,9 masks,481401 proof-consumed
branches; true tag remains included despite polluted native output. A healthy-
only consumer premise is disproved directly by true range outside faulty
row's nominal shell (finite optimizer/interval relaxations cannot repair that
premise). Healthy-only projection requests, foreign receipt version and
invalid evidence refuse. Box budget0 keeps the valid physical frontier for
ALL9 masks and all physical branches, not omitted branches with zero risk.
No actual joint recovery success is claimed before W3.

W2 independent deterministic oracles: literal raw perturbations/reintegrations
for all six axes across two native PIMs sharing an endpoint, frozen current
UWB basis concatenation, QR versus SVD protected response and a two-parameter
superposition check. The separate integrated-accel/UWB 1D frozen alias has
`A=[1;1]`, `B_U=[1;0]`, `B_I=[0;dt²/2]`: theta=(2m,1600m/s²) yields a2m
protected shift, zero joint residual, each local1-row fit also zero residual,
but individually nonzero joint parity directions. Existing response
classification reports dangerous rank1/q2, and Gaussian joint PL is unbounded.
This is a fixed linear small-oracle detectability counterexample, NOT an
actual raw-likelihood/trajectory guarantee or a claim that the raw detector
misses this strong acceleration. Raw physical coverage never borrows its
single slopes. Geometry/nullspace handling will remain separately labeled.

Failure accounting: W2 first check invocation passed both joint32-commit
fixtures, then failed its response-superposition oracle due to an Eigen
lazy-expression holding expired QR temporaries. Concrete QR/vector lifetimes
fixed the oracle; only the affected linear response/cancellation branch reran
and passed. No source sigma, witness, threshold, AL or math envelope changed;
no native epoch was aborted by that oracle assertion. W1 generation checks
remain passed. Final frozen native joint acceptance remains pending W3/W4.

W3 actual integration PASS: one Sc3JointCoordinator and existing FdeManager
research consumer evaluate KEEP/UWB/IMU/both roots (18 when triggered), each
against all481401 physical branches via all9 projections. Native temporary
rebuilds preserve selected historical UWB rows, remove the complete SC2 IMU
support closure, use original observed bias and exactly two GN rounds. No
candidate performs backend update; the selected native transaction commits
once. Full historical UWB source removal starts at epoch1, not at injected
labels. Current replacement and historical deletion have distinct receipt
semantics. Gaussian functional evidence selects actions; new raw protection
never upgrades the old Gaussian recovery covariance or production risk ledger.

W3 actual 26-epoch A/G checks:52 scheduled/attempted/committed,0 aborted,
52 qualified. Both simultaneous at15–17, IMU bridge15–25 including shared
endpoint1.200s, native IMU returned26. At15 A H/V=3.4128/6.11873m,
G=3.41189/6.11471m;14 historical UWB groups removed. All18 roots retained
when alarmed. Independent six-axis response/support oracle rerun PASS after
lazy native material binding; missing native PIM provenance refuses response.
Fixed G runner snapshot:16 scheduled/attempted/committed,0 aborted/16 valid.

W3 failures retained in denominators: initial native unit run scheduled52,
attempted16, committed15, aborted1, unattempted36; targeted debug run
scheduled16, attempted16, committed15, aborted1. Historical selected7-row
replacement needed6 rows when another hypothetical root removed a different
anchor. Fixed selection to preserve actual selected rows minus that anchor;
no witness/law/threshold changes. Audit also fixed UWB search trigger to use
explicit statistic>detection threshold because the reused WLS result does
not set its alarm flag. Across these W3 native attempts including repaired
52 and fixed16: scheduled136, attempted100, committed98, aborted2,
unattempted36. Linear oracle has one actual commit and one discarded pending
transaction separately, never treated as a failed navigation epoch.

W4 preregistered freeze:320 stream epochs,12 streams: joint A/G off/on30
(120), IMU-only A/G on30(60), staggered on30, redacted A on30,
new-pool UWB U1/U2 on19/21(40), original SC1 U1/U2 on19/21(40).
No seed/amplitude search.290 scored,30 redacted. Acceptance and final affected
regression remain pending W4. No SC3 completion or SC4 work claimed here.

### SC3 W4 acceptance and SC4 handoff

Functional freeze SHA: `90c58a6`; W1 `bd5fe70`, W2 `7a805ee`. Final W4
adds refusal/receipt assertions and source-scope admission guards, preserving
all fixed numeric inputs. Commands: `python3 tools/sc3_joint_research.py
checks --build` and `python3 tools/sc3_joint_research.py streams`.

Both frozen runs completed320 scheduled/attempted/committed/finite/qualified
raw positions each,0 aborted/refused. First run omitted contracted/fallback
counters in its harness; the affected fixed matrix was rerun only to recover
these counters. This is640 actual commits,580 scored positions,60 redacted;
not640 independent risk trials. Corrected run:320 CONTRACTED_VERIFIED,
0 fallback. Both runs have the same per-stream late counts below. Geometric
qualification here is spatial coverage; late rows are NOT timely protected
publications. All application/formal/publication fields remain0.

| Frozen stream | Epochs | Contracted | Fallback/refused/abort | Late (>50ms) |
|---|---:|---:|---:|---:|
| joint accel off/on |30/30|30/30|0/0|0/16|
| joint gyro off/on |30/30|30/30|0/0|0/16|
| IMU-only accel/gyro on |30/30|30/30|0/0|16/16|
| staggered on |30|30|0|19|
| joint accel scoring-redacted |30|30|0|16|
| new-pool UWB1/UWB3 |19/21|19/21|0/0|3/3|
| original SC1 U1/U2 |19/21|19/21|0/0|13/16|

Scoring redaction leaves every online state, action, root count, PL, raw
source, risk and receipt exactly identical. Before faults, on/off native
states match exactly. Both overlapping families actually remove anchor0
and full IMU support at15–17, bridge15–25, rejoin26–30 and retain valid
all9/all481401 raw coverage. Staggered UWB12–14 precedes IMU15; historical
UWB removal remains in the actual single graph, and the physical bank/event
never resets. New-pool UWB-only fixtures actually exclude anchor0 at15
without IMU removal; original SC1 U1/U2 remain valid and never widen their
old raw inertial bound.

| Actual joint snapshot | HPL/VPL m | PE_H/PE_V m | Native action |
|---|---|---|---|
| accel15 |3.412803/6.118728|.123237/.100731|both,14 historical UWB removed|
| accel17 |3.617849/6.809397|.027318/.013716|both|
| accel25 |2.846389/5.222348|.023760/.125685|IMU bridge shared tail|
| accel26 |2.845210/5.313355|.018512/.040100|IMU rejoined|
| accel30 |3.000259/5.208363|.119962/.026270|four subsequent outputs|
| gyro15 |3.411885/6.114706|.122326/.096709|both,14 historical UWB removed|
| gyro17 |3.617971/6.808945|.027270/.013263|both|
| gyro25 |2.846424/5.222090|.023751/.125427|IMU bridge shared tail|
| gyro26 |2.846523/5.314759|.020881/.041503|IMU rejoined|
| gyro30 |2.903242/5.364836|.043575/.130203|four subsequent outputs|

First fixed-run mean/median/P95/max ms (no concurrent build): accel candidate
101.38/142.09/231.16/254.58, final bound .600/.574/.796/1.046, full service
104.99/146.13/236.07/260.67; gyro candidate141.76/210.26/336.60/376.93,
final bound .572/.566/.671/.715, full145.15/214.26/342.12/382.35.
Source initialization is separate,87.11ms accel/59.34ms gyro. Service includes
monitor/search/copies/rebuilds/candidate proof and consumer reproof/native
commit/final issue/score/retained-ID collection, excludes CSV serialization
and I/O. These are PC service samples, not hardware TTA/WCET. Full overhead
includes the native commit; no expensive rebuild is omitted as diagnostic.
Most cost is candidate work. No SC4 performance claim.

Raw position protection is not a full navigation accuracy guarantee: accel
velocity/rotation maxima1.8923m/s/1.1982rad, gyro3.5772m/s/1.7784rad.
AL remains postprocessing: e.g.10/10m numeric passes all30 A/G positions,
while2/3m passes0; application availability remains0 regardless. The risk
amount, actual FDE action and PL never depend on this numeric AL filter.

| SC3→SC4 field | Actual binding / limitation |
|---|---|
| requested |H0/UWB/all six raw IMU axes/UWB+accel/UWB+gyro; constant single episodes, declared onsets/durations, any amplitudes, overlapping or staggered|
| implemented |481401 exact physical identities;1400 stored modes; complete18 numerical action roots on alarm/held isolation; raw support closure and native UWB replacements|
| verified mathematical scope |full requested raw position union on original6421-source E; full-state independent propagation, all9 UWB masks, any selected center/action|
| verified native recovery subdomain |registered anchor0+accel-x20 or gyro-x1 at.705s/500ms,3 UWB epochs at15 or stagger12; real backend/history/tail/rejoin; UWB1/3 and IMU-only actual cases|
| missing native scope |other axes/amplitudes/onsets/long-term navigation not empirically qualified; no silent removal of gyro or other requested physical categories|
| online proof dependencies |matched original initial law/numeric process certificate/raw prefix/sealed full8-row range evidence/full physical envelope/all9 projections/action census/final actual receipt+tag|
| oracle only |six-axis finite difference/QR/SVD and cancellation frozen model; neither supplies online raw risk|
| resource |mode1400,action18,range256; physical recipe481401 distinct identities; capacity1399 or17 refuses globally; range0 retains every physical frontier|
| worst declared work |18 candidate copies; up to17 retained-graph optimizations×2 GN rounds×8 iterations; all9 range projections for each producer+consumer proof and final issue, at most333 projection calls per alarmed epoch; no481401 dense numeric fit claim|
| task termination |at most50epochs/2.5s from original valid initial boundary; task-internal history persists; no initial carry-in or cross-task renewal|
| conditional task risk |sup joint deterministic fault/action/issued k≤50 PE>PL ≤3.7375139072537422e-7; same E, no new charge per root, no independent prior product|
| detection/continuity |raw IMU necessary test false-alarm conditional E bound inherited; weak faults can be unmonitorable; functional UWB/Gaussian post-test has no new raw false-alarm/MD/HMI theorem|
| omission/deployment |out-of-domain initial/source/motion/horizon/multiple episodes/axes unknown or refused, never risk0; hardware priors/calibration/HMI/infinite horizon OPEN|
| lifecycle/failure |missing original x0/irreversible marginal history rejects recovery; proof can independently protect current position when valid; expired evidence/invalid source/unknown finite reference refuses; no second commit repair|
| output schema |CSV action/support/history/root/receipt/state/tag/proof/count/risk/timing fields in runSc3Joint; raw and functional Gaussian levels separate; formal/publication/application0|

Negative coverage: actual wrong healthy IMU deletion at14 and wrong healthy
UWB anchor1 deletion at15 retain the true joint fault explanation under the
all9 raw bound; raw-prefix/frame/receipt/version/update-count/state-expiry
mutations refuse. Physical envelope51 refuses renewal. Existing shared
range kernel mirror/singular/slab/lattice/frontier tests protect both mirror
branches and are part of final regression. Dangerous joint nullspace oracle
returns unbounded Gaussian PL. Source-domain two-axis refusal and weak
undetected IMU cases remain covered by upstream SC2 regression, without
upgrading them to joint recovery positives. Resource failures never remove
physical explanations or count unevaluated numerical branches as risk0.

The final SC3 acceptance and completed regression are recorded below.
This earlier W4 checkpoint was superseded by the completed audit.

Fixed off/on comparison: accel horizontal PE mean/max .1900/.6300m →
.06519/.15466m and vertical .1692/.5787m → .05525/.13432m. Gyro horizontal
.1273/.5338m → .06520/.19819m; vertical .04938/.12020m → .06141/.18218m,
so vertical gyro accuracy worsens despite all raw coverage passing. This
precludes a general navigation accuracy improvement claim. A/G on HPL
mean/median/P95/max2.9583/2.9055/3.4144/3.6178m and
2.9551/2.9029/3.4140/3.6180m; VPL5.4494/5.3837/6.1107/6.8094m and
5.4572/5.3821/6.1083/6.8089m. Zero observed violations are implementation
checks, never a statistical proof of the conditional task-risk amount.

Each indexed physical hypothesis also carries explicit original source-event,
branch-constraint recipe and separate NOT_CONSUMED bound/local-PASS retention/
native-recovery-unqualified statuses. Generation cannot qualify a physical
hypothesis; the receipt-bound all-branch consumer supplies the coverage result.
Functional action statuses are distinct from these physical evidence statuses.

Final affected regression: sc3_joint_checks, sc2_imu_checks,
raw_range_checks, hybrid_raw_checks, hybrid_checks, epoch_raim_checks and
fused_epoch_checks all PASS (last two with config/fde_next.yaml). Final SC3
native fixtures84 scheduled/attempted/committed,0 aborted,84 raw qualified;
plus one diagnostic setup commit and one pending oracle discard. Combining
SC3 trajectory attempts across W2/W3/W4, including both frozen runs and both
W3 failures:892 scheduled,856 attempted,854 committed,2 aborted,
36 unattempted. Diagnostic response oracles are separate:four invocations,
one initial Eigen harness assertion failure and three passes, one setup
commit each; the failed invocation's read-only pending transaction was
abandoned on process exit, three successful invocations discard it. Old-stage
regressions are listed separately and not added as new SC3 evidence trials.
Build warnings remain pre-existing/compact-statement style warnings; all
final targets compile/link successfully, no failed build hidden as success.

| Clause | Actual acceptance evidence | Result |
|---|---|---|
| SC3-A01 |requested/implemented/math verified/native verified/missing all explicit; actual SC2 bridge interface and original boundary|PASS for declared scope|
| SC3-A02 |481401 bijective physical identities,1400 source modes, common source event, lazy native shared-amplitude support closure and independent statuses|PASS|
| SC3-A03 |local PASS preserves full product; KEEP coverage all9, alarm roots18 with trusted census and consumer reproof|PASS|
| SC3-A04 |new independent-full-state plus branch-valid shells union; no healthy-IMU/healthy-UWB joint inheritance|PASS|
| SC3-A05 |six-axis actual shared-PIM response versus literal reintegration/QR/SVD; dangerous joint cancellation unbounded; shared geometry mirror oracle|PASS|
| SC3-A06 |same6421-source E, sup deterministic h/theta/action/center, task-wide raw PE event; omitted/application risks explicitly unknown|PASS conditional theorem|
| SC3-A07 |actual UA simultaneous15–17, bridge through25, restored26 and four further raw tag outputs27–30|PASS native registered accel witness|
| SC3-A08 |actual UG same complete lifecycle; gyro category retained, full native-axis expansion remains unknown|PASS native registered gyro witness|
| SC3-A09 |actual wrong healthy IMU/UWB deletion; history removal, shared tail25, original-bias IMU rejoin26; raw union never drops real explanation|PASS|
| SC3-A10 |one actual backend, selected transaction one update, receipt/source/time/tag/version/current replacement/history slots checked; Gaussian/raw independent|PASS|
| SC3-A11 |mode1399/action17 refusal, box0 all-frontier fallback, prefix/frame/receipt/expiry/domain refusals, full service costs, AL postfilter|PASS|
| SC3-A12 |all attempt/failure/late denominators, original affected regressions PASS, SC4 handoff and retained CLI/config/source/unique conclusion, W1–W4 local commits|PASS after local W4 save|

SC3 completion status: `JOINT_FDE_FUSED_BOUND_VERIFIED_FOR_SCOPE` for the
mathematical physical domain and the separately stated actual native witness
subdomain above. Both required joint categories have real successful results;
this is not an accel-only substitution. Broad native recovery for every
requested axis/amplitude/onset remains OPEN, so unrestricted
`JOINT_TARGET_COMPLETE` is NOT asserted. No SC4 or whole-system completion is
claimed. `SC4_PERFORMANCE_OPEN`, `APPLICATION_AL_UNQUALIFIED` and
`DEPLOYMENT_UNQUALIFIED` remain. Production ABI layouts/default behavior,
original source law, FGO, risk and AL are preserved; new research member
functions link only through fde_research and require explicit SC3 opt-in.

Own temporary logs/CSV and frozen stream directories are removed after this
summary. No old-stage or user artifacts are deleted. No new plots, binaries,
backup registries or repeated manifests are retained; no remote push.


## SC4 — continuous system and realtime/application evaluation (in progress)

Entry actual local clean HEAD `5b7d960`; no reset, old-stage redo or remote
push. SC4-spec-1.0 and COMMON_REQUIREMENTS bind the actual SC3 handoff.
Requested H0/UWB/all six raw IMU axes/UA/UG remain named. Raw mathematical
qualification remains SC3's full481401 physical product/all9 current UWB
projections/original6421-source50epoch event; actual native recovery remains
registered accel-x/gyro-x witnesses, not an all-axis navigation qualification.
The old SC3 checkpoint sentence above is corrected to point to its completed
final audit; this is a documentation correction, not a new SC3 claim.

W1 entry/runtime preregistration: config/sc4_system.yaml explicitly binds
source5b7d960, original scope, single-thread runtime, core40/full50ms,
queue64 and max120 replay epochs. No production contract/source sigma/risk/AL
is changed. New `epoch_raim_runner CONFIG CSV SCENE|record:CACHE system
on|off COUNT reference offline|paced` routes one system entry. Matched replay
uses existing NoisyMotionScenario and actual Sc3JointCoordinator/backend;
record replay consumes existing canonical benchmark measurements and causal
initializer, never GT or simulation draw certification. Both adapters use
one FGO instance; record geometry/source insufficiency remains unprotected.
Illegal source IDs, timestamps, missing shared samples/batches, expired
numeric evidence and insufficient IMU tail have explicit admission reasons.

| Final field / consumer | Required computation, retained |
|---|---|
| functional alarm/search/posttest |raw IMU necessary interval bank; current Gaussian UWB WLS; all18 action roots when triggered/held; no local PASS physical pruning|
| candidate numerical center |selected retained graph, original-observed-bias raw PIM reconstruction, full support/history deletion and native UWB replacement, exactly two GN rounds|
| candidate/final raw proof |original-initial independent full-state physical envelope, sealed complete original batch, all9 shell intersections, full481401 logical product; source/time/version/prefix/census checks|
| selection/commit |existing FdeManager research consumer reproof, trusted complete action census; cardinality/statistic order; one native transaction/receipt; no AL ranking|
| rank/boundary covariance diagnostic |boundedRecovery healthy_graph QR/Marginals/fullStateBridge/local-domain status; SC3 numerical selection reads only rebuilds/finite objective/reference, NOT these diagnostics; potential W3 profile omission requires equivalence evidence|
| old Gaussian H/P / independent oracles |SC3 never consumes old OnlineGaussianHistory/preUwbPrediction; not silently removed from a dependent pipeline; legacy paths and strict response/range oracles retained|

Runtime clocks use steady_clock, independent sensor timestamp domain:
measurement/replay release, processing start, first observed raw alarm,
selected candidate, actual commit, proof completion and in-process output.
Stage timings include ingest, native prepare/evidence extraction, every
candidate/rebuild/reproof, commit, final proof, transaction cleanup and packet
packaging. Replay age is release-to-output (same-domain replay measurement
mapping), not age of a historical bag relative to current UTC. Offline replay
maps each available batch to its actual processing release, without negative
ages. Paced replay maps original sensor intervals onto steady_clock. Hardware
capture/transport/TTA are unknown. Source parsing/generation/certification/
bootstrap/backend initial setup is measured separately as cold initialization;
no cold-start realtime claim. CSV diagnostic logging is outside in-process
output packaging and will be accounted as runtime overhead/queue service;
there is no production publication. Queue/overload lifecycle is W2, not yet
implemented or claimed by W1.

SC4 mathematical contract remains finite50, no initial carry-in/renewal.
Explicit unprotected stress generation may extend input to120 epochs, but
certifyRawGenerator consumes only the original≤501 sample task prefix;
extra sources do not acquire an inventory/risk claim. W2 must prove operational
carry and expiry behavior before this becomes a continuous-system result.

Pre-registered final performance workloads: config/sc4_performance.yaml
normal SC3_U0 and heaviest successful SC3_G (selected from actual SC3 cost,
not fastest observed frame),50 input epochs:15 warmup +35 measured. Fault
onset is16 for both UWB and gyro so measurement includes actual first alarm,
500ms support and rejoin. This shifted witness has no inherited native
success; local actual attempt must validate it before freeze. Resource/carry
stress separately registers late fault48,60 input epochs and fixed lag80 in
config/sc4_carry.yaml; it is NOT an equivalent performance profile or longer
protected mathematical scope. Both profiles retain original source weights.

Build accounting so far: first W1 build refused missing direct config/backend
header declarations; second refused optional lever→vector assignment. Both
are compilation failures before any native epochs, not hidden navigation
abort counts. Fixes add explicit include and value_or for an absent lever.
W1 actual validation/local save remain pending.

W1 verification PASS after the two compile fixes. Deterministic adapter/source/
time checks plus two actual single-commit healthy outputs pass. The initial
check harness assumed graph version starts0; actual initialization consumes
version1. It failed after one valid native commit,0 native abort,1 scheduled
unattempted. Corrected check verifies actual proof receipt version and +1
monotonicity, not an invented initial version. Repaired check commits2/2.
The shifted heavy gyro witness local snapshot commits17/17, all raw qualified:
actual joint exclusion at16/17,18 evaluated roots; core/full at16
322.1125/322.1375ms and17 230.9248/230.9494ms. Cold source/init38.41ms.
Across W1 native check/snapshot attempts:21 scheduled,20 attempted/committed,
0 aborted,1 unattempted; no failure erased. Input negative fixtures have0
native commits by construction. These local samples are not a realtime result.
Platform observed WSL2 Linux6.18.40.1 x86_64, Intel Core Ultra5 125H; runtime
pins OMP/BLAS/MKL and TBB to1. W1 saves actual source+time interface; W2/W3/W4
and SC4 acceptance remain active, not completed after this local save.
