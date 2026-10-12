# System completion — SC1

Status: **MEASUREMENT_CONSTRAINED_RAW_BOUND_VERIFIED** — SC1 only.
Starting HEAD `0e88303f9f22b6c47ecb440042f3f513780caf8e`, clean tree.
FGO, FDE, raw-source-rw-v1, sigmas, risk, AL and production publication
remain the original contracts. SC2–SC4 are outside this work.

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

## SC2 — IMU fault scope and legal recovery (in progress)

Starting local HEAD `50b7582`, clean tree, SC1 retained. No SC2 completion is
claimed yet. Legacy I2 (`config/fde_next.yaml`) uses accel-x +20m/s² on raw
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
