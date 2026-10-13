# SC5 finite rolling integrity

Contract: `docs/requirements/sc5_rolling/ROLLING_INTEGRITY_HANDBOOK.md`, RI01–RI16.
Entry HEAD: `d668bcb3f7b48df854fdae1882b937fd804185cf`; clean working tree;
no repository/ancestor AGENTS.md found. Local GTSAM is **4.2a5**, Release,
system Eigen, TBB; existing build `/home/mint/ws_fusion_uwb/build/uwb_imu_pl`.
No old campaign is repeated. Production defaults, source sigmas, AL and
publication switches remain unchanged. SC5 is a research opt-in.

## W0 preregistration

Native timestamps use logical epochs and lag L−1 (unless nominal physical
lag is explicitly enabled, which SC5 will reject). Source inspection:
`backendUpdate` records actual disappeared keys and boundary factor slots in
CommitReceipt before retained metadata is pruned. Expected x0 exit is update
L; this expectation must be checked against actual keys, not used as evidence.
Current boundedRecovery refuses absent x0; current Sc2IndependentBridge
recomputes from t0. Neither qualifies as rolling recovery/transfer.

Frozen matrix, before observing SC5 results:

| Row | Played / mission | FGO / integrity | Input |
|---|---|---|---|
| R0 | 200 / 200 | 30 / 30 | healthy, seed20261010 |
| R1 | 200 / 200 | 30 / 30 | SC3_A, original20 m/s² accel and2.25m UWB, onset61 |
| R2 | 200 / 200 | 30 / 30 | SC3_G, original1rad/s gyro and2.25m UWB, onset61 |
| R3 | 80 / 200 | 30 / 30 | SC3_A onset20, artificial recovery admission delay55 |
| R4 | 70 / 200 each | 20/20,40/40,30/10 | healthy |
| R5 | small fixtures | parameterized | scope, source, resources, receipts, recovery horizon |

R3 declares recovery_extra_epochs=30: a60-epoch bounded buffer retains onset20
at55; active FGO begins26 and its onset factors have already marginalized.
Artificial delay is a recovery test, not an automatic detection claim.
Main streams declare extra20. One UWB constant episode (1/3 epochs), one IMU
constant single-axis episode (50/500ms), all onsets and unbounded amplitudes,
no mission-initial carry-in. Window carry persists. N(T)=21+128T independent
Gaussian primitive coordinates; logical hypotheses=1+28T+192T². Compact
recipes must represent the product without dense numerical candidates.

Source/event radius is registered before measurements. New path will use a
certified exponential tail bound; all source consumers must share it. Legacy
50/6421/q7 path remains unchanged. Numeric candidate weights are metrics;
raw protection uses absolute sets. Application/deployment/time-uniform and
untested navigation recovery dimensions stay unqualified.

W0 verification: actual HEAD/status, local GTSAM config, native timestamp,
receipt/prune order, raw source count and SC3/SC4 handoff inspected directly.
RI01 baseline PASS; remaining clauses OPEN until implementation and tests.
Core status: **RAW_TRANSFER_OPEN / HISTORY_RECOVERY_OPEN / RISK_EXTENSION_OPEN**.

## W1 window/lifecycle

`sc5_rolling_checks` PASS: each30/30,20/20,40/40,30/10 runs70 native
epochs and70 independent control epochs,560 commits total,0 native aborts.
All full navigation states are exactly equal to observation-free control.
Actual x0 exits at30,20,40,30 respectively; actual marginalized key totals
123,153,93,123. Post window counts are min(k+1,Lf); proposal checks actual
disappeared keys, full selected source groups and transaction/version receipt.
Wrong/overflow/conflicting windows, Li>Lf, stale receipts and hot resize refuse.
First build failed for a missing Symbol include, repaired; first invocation
loaded an older library from the ambient environment and failed before any
native attempt. Explicit local LD_LIBRARY_PATH resolves the correct build.
Lifecycle alone does not certify raw transfer or historical recovery.

Reproduce: build target `sc5_rolling_checks` in the existing build, then run
with `LD_LIBRARY_PATH=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib:$LD_LIBRARY_PATH`.

## W2 operator and fixed-model evidence

`sc5_math_checks` PASS: independent dense SVD of12 original white rows checks
state mean/covariance,8 residual dof, residual energy, joint two-column
UWB/IMU response, full residual Gram including nonzero cross term, and RHS
after retiring two state columns. One joint leaf is used: old mutually exclusive
leaves are not misinterpreted as coexistence. A shared ADC latent with one
prior has a different covariance from the deliberately duplicated-source model.
100-digit independent rational endpoint arithmetic verifies continuous reach.
Small fixtures check p→v contraction→future p, small velocity carry, rotation
lever uncertainty, RW carry, changed parent dependence, and all9 nonempty
branches retained when the box budget is zero. No full-prefix statistical
interpretation is attached to the raw representation.

Operator proof (absolute state, conditional declared physical motion domain):
initial tangent translation has norm≤||rho|| since SE(3) V is an integral of
rotations. Rotation displacement operator norm≤min(2,||phi||); matrix entries
are enclosed by normalized quaternion interval arithmetic plus this radius.
For each actual edge dt=1/20, bounded acceleration gives p1=p0+dt v0+rp,
|rp_j|≤A dt²/2, v1=v0+rv, |rv_j|≤A dt. Angular-rate norm gives each matrix
entry displacement≤w dt; intersecting[-1,1] retains every rotation. Ten RW
increments have support within ±10 q sigma sqrt(1/200); Cartesian hull loses
correlation conservatively. Every tag/body/rotation contraction is an inverse
of p_tag=p+Rl with interval remainders. Every backward dynamics intersection
uses a necessary rearrangement of the same physical relation. Thus no feasible
trajectory is lost. Projection retains coordinate hulls of p/v/R/bias, not
independent Gaussian marginals. Directed Boost interval arithmetic and explicit
finite-domain checks cover rounding; a runtime rounding witness rejects an
unsupported arithmetic environment. Rank/gauge deficiencies refuse Gaussian
qualification; frontier exhaustion keeps all raw outer branches.

W2 has no nonlinear native rolling/recovery claim yet. Reproduce target
`sc5_math_checks`, same local library environment as W1.

## W3 actual nonlinear rolling

One70-epoch healthy native stream:70 scheduled/70 committed/70 qualified,
0 native abort; x0 actually disappears30,123 marginalized keys. Proof keeps
at most29 explicit raw UWB batches plus its boundary,501 observed IMU replay
samples; at51 the raw replay first ID is11 (initial raw retired), proof material
first endpoint ID211, separator22, parent50. All70 post-scored errors are inside
their actual committed tag bounds. At30 H/V=4.32087/6.79909m; at51
4.36458/6.86485m. AL is not a qualification gate. Performance remains open.
The first source-domain check erroneously used a sum of acceleration maxima
instead of its vector norm and refused before any epoch; corrected to a
directed Euclidean bound. This failed admission is not a native success.

`sc5_rolling_checks boundary` PASS2/2 native commits: sealed full batch,
unregistered scope, future transaction and old proposal refuse; post-commit
proof failure never calls a second update. Changing future raw UWB/IMU and
future numeric admission leaves current bounds/separator bitwise unchanged.
Initial p/v/R raw observation is not stored in the proof after construction;
the monitor retains only the6 bias source-recipe centers. Source mapping
certification processes11-sample arrived blocks with carried RW rounding,
and supplies fixed1e-10 bounds, never future max residues to online decisions.
Generator buffers are separate simulation-provider memory.

Each proposal starts from the preceding certificate's absolute joint box,
propagates only its bounded explicit window, contracts all9 UWB mask branches,
performs two forward/backward necessary-relation passes, and projects to actual
integrity cutoff. Finalization verifies actual native keys/receipt, then
publishes the parent-linked set and releases retired batches. Repeated set
intersections are deterministic/idempotent evidence, not additional Gaussian
weights or fresh probability events. IMU faults are outer-covered by declared
fault-independent physical reach. The task fault recipe remains1 UWB/1 IMU;
per-epoch mask relaxation is explicitly an outer superset, not a newly allowed
episode per window. History phase persists after fault end.

Reproduce: `epoch_raim_runner config/sc5_rolling.yaml OUTPUT SC3_U0 rolling off 70`.
Raw transfer main algorithm is implemented; T200 and actual FDE remain OPEN
until W4/W5 validation. Dynamic source-event plumbing enters here because all
operators and monitor constraints must consume the same predeclared radius.

## W4 bounded native recovery

Actual UA75 and UG75 witnesses PASS75/75 raw/native each: x0 absent at61,
joint UWB/IMU exclusion, actual complete bridge pairs through71, and actual
original-bias CombinedIMU reintegration/rejoin72–75. UA61 removes39 groups,
including1 actual LinearContainerFactor boundary; candidate uses exactly two
bounded8-iteration GN rounds on live keys. The first possible IMU onset52 is
a conservative raw monitor union, not the generator's hidden true label61.

Delayed onset20, isolation55, replay60:80/80 receipts/raw outputs PASS. At55
FGO separator26, actual original fault factors20–22 have marginalized; one
polluted numerical boundary plus58 active affected groups are really removed.
Three recorded regularization factors replace the discarded prior at26;
ledger marks their explicit source/metric and prior replacement relation.
Old boundary pointer absence, actual removed slots, full metric receipt and
all historical removals are checked.56–60 genuinely rejoin IMU. At55 actual
tag horizontal/vertical error .11096/.10275m; no full15D accuracy claim.
Numerical metric seeds use coordinate midpoints and nearest SO(3) rotation
from certified raw capsules; they need not be jointly feasible points,
not independent observations; their weights retain original nominal metric
sigmas and never narrow raw proof. The optimizer visits only live keys, never
reconstructs epoch0..k. Capsules/raw tail are explicitly bounded by replay.

The existing strict slot binding initially failed at55 because the earliest
historical replacement pair itself marginalized during that same update.
A narrow explicit `commitEpochRollingResearchV1` now accepts only a complete
rolling action and a replacement tied to this transaction whose actual keys
marginalized; its ledger lifecycle is Marginalized, not an invented live slot.
Legacy commit signatures/layout/default binding remain unchanged. Two failed
80-scheduled runs occurred: each54 emitted protected receipts, one additional
native mutation terminal at55,25 unattempted. The second mistakenly launched
before the new runner finished linking, thus used the old interface. Neither
failed backend received a second mutation. Corrected80 run is separate.

Out-of-buffer onset20, extra0/replay30, isolation55:60/60 native/raw PASS;
all6 epochs55–60 return `BOUND_VALID_BUT_RECOVERY_UNAVAILABLE`, no repair,
no false FDE_SUCCESS. Bound and recovery are separate decisions. The buffer
gate checks the preceding shared endpoint, not only the first bad sample.
Reproduce delayed/outside by overriding onset fields to20 and extra to30/0
in a transient SC5 config, runner `SC3_A rolling on 80|60 55`.

W4 stream attempts:75+75+80+60+80+80=450 scheduled;
75+75+80+60+54+54=398 protected receipts;2 additional terminal native
mutations;50 unattempted. No failed row is recategorized as successful.

## W5 finite task theorem and T200

One predeclared event owns N(T)=15+6(10T+1)+60T+8T=21+128T standard-normal
coordinates. Compact source IDs match the legacy6421 inventory exactly at50;
the original50/q7/2·6421·2^-35 path is unchanged. New event selects integer m
with exact binary upper 2N·2^-m≤budget, then integer q with5q²≥7m. A rational
Taylor partial sum proves exp(7/10)>2, hence ln2<7/10 and
2N exp(-q²/2)≤2N·2^-m.100-digit independent checks PASS for11 task lengths
1,2,7,49,50,51,100,200,201,777,10000. Large test tasks use compact recipes,
not million-entry inventories. At200 N=25621,m=36,q=8, booked upper
**7.4566924013197422e-7**; exact Chernoff expression≈6.48937e-10 is checked
but not substituted for the conservative binary amount. Every raw monitor,
theta interval, initial set, ADC/RW support and range shell consumes this same
event radius. Mission cap10000 is explicit; larger missions refuse. Numeric
adapter domains can additionally refuse an arrived batch; invalid future
certification never changes an earlier accepted batch or radius.

Let F_k be the exact feasible relation under the declared single UWB and IMU
episodes, source event E_T, numeric mapping certificates and physical motion
domain. W2 initial inclusion and propagation establish true absolute states
inside each box. For each allowed UWB episode its current row omission is
among all9 retained branches; fault-independent physical reach contains every
IMU branch. Every necessary tag and dynamics contraction retains the true
trajectory. Each Cartesian separator projection therefore contains π_B F_k,
and its next-window use preserves this invariant by induction. Shared latent
correlation can be outer-projected, never counted as a new independent draw.
Bias propagation additionally outward-inflates2e-10 per edge for numerical
mapping support; it does not reset bias or spend probabilistic risk.

On E_T every accepted, receipt-bound actual tag center is protected by the
componentwise supremum over its absolute feasible tag outer set. This holds
for **all data-dependent actions and centers at once**, including an incorrect
FDE selection: neither the raw set nor the event assumes the chosen label is
true. Horizontal norm and vertical absolute bounds are outward-rounded.
Thus for every deterministic allowed h,theta,
P(any issued k≤T exceeds HPL or VPL)≤P(E_T^c)≤budget, and the supremum over
those faults obeys the same inequality. There is no extra candidate union,
window reset, random-linearization Gaussian assumption, or risk multiplication
by the7,685,601 logical fault hypotheses. Simulation PE checks validate the
implementation; they are not a probability estimate or a hardware HMI proof.

New numerical proposal seal compares actual interval endpoints, source/window
fields, phases and IDs, not merely a hash or valid flag. Tampered small sets,
forged FDE receipts vs actual ledger, stale/future proposals, whole-bank
capacity, Gaussian rank deficiency, pre-update retry and injected post-update
failure PASS. The latest boundary test has3 successful native commits and1
deliberate terminal mutation; retries preserve149 introduced sources at1 and
the original event amount. No second mutation follows a post-update failure.

`python3 tools/sc4_system_research.py rolling --main-only`:600 scheduled,
600 native receipts/finite qualified outputs,0 aborts. Each stream actually
marginalizes513 keys, x0 exits30.51/90/150/200 are qualified in all3; source
prefixes6549/11541/19221/25621, separator22/61/121/171, parent50/89/149/199.
Their event amount never changes. UA/UG automatically isolate61, bridge
through71, actually rejoin72–75 and continue through200. Final H/V meters:
healthy4.31743/6.86663, UA4.31536/6.86788, UG4.31225/6.86758.
All600 post-scored PE are inside their signed PL. Application status remains
APPLICATION_AL_UNQUALIFIED;2/3 and5/5 pass0 of200 each,10/10 pass200 each,
without modifying native decisions/states/sets/PL/risk.

Main full service mean/median/P95/max ms: healthy27.8249/29.4941/31.2860/54.5562;
UA27.7894/29.5208/32.1623/39.6093; UG27.8853/29.4816/32.2305/38.3583.
Healthy core40 misses2/200, full50 misses1/200; UA/UG miss0/200. Cold source
initialization221.25/208.15/209.25ms is separate. Actual `/proc/self/statm`
resident maxima23524/23684/23848KiB; `getrusage` maxima can include inherited
pre-exec high-water marks and are not substituted for resident memory.
ROLLING_PERFORMANCE_OPEN remains. W6 variants, delayed final and relevant
legacy safety regression are still required before RI01–RI16 completion.

## W6 final acceptance

Implementation identity: **4d7d2bd**. W6 adds an explicit SO(3)/lever-arm
inverse witness and this report, without changing or repeating the three
frozen T200 streams. W6's doc/test commit is the final local HEAD (`git log -1`);
no self-referential hash is invented here. Actual work-package saves:
W0 `70caf0f`, W1 `32c1ced`, W2 `ee9cfc3`, W3 `f399cb6`, W4 `df921d1`,
W5 `4d7d2bd`. No push.

Final matrix: **953 scheduled / 952 native receipts / 952 finite qualified
tag outputs / 0 unexpected native abort / 1 expected expired admission**.
Earlier development failures remain in W1/W3/W4 accounting above. An initial
new-math-target invocation preceded CMake reconfiguration and failed before
any native epoch. Expected negative fixtures/injected mutations are separate
from positive stream qualification.

| Row | Played/mission | Actual FGO/proof | Native/qualified | x0 exit | Marginalized keys | core40/full50 misses |
|---|---|---|---|---|---|---|
| R0 healthy |200/200|30/30|200/200|30|513|2/1|
| R1 UA |200/200|30/30|200/200|30|513|0/0|
| R2 UG |200/200|30/30|200/200|30|513|0/0|
| R3 delayed |80/200|30/30; replay60|80/80|30|153|0/0|
| R4 follow20 |70/200|20/20|70/70|20|153|0/0|
| R4 follow40 |70/200|40/40|70/70|40|93|14/0|
| R4 explicit10 |70/200|30/10|70/70|30|123|0/0|
| R5 outside |60/200|30/30; replay30|60/60; recovery unavailable55–60|30|93|0/0|
| R5 expiry |3/2|30/30 warmup|2/2; epoch3 refused|not reached|0|0/0|

Every admitted row checks actual state keys, unique native version/update,
separator/parent, bounded material, actual raw read range, source prefix,
persistent phases/event amount, PE inside PL and closed production flags.
The T200 event unit directly rejects prefix201; native T2+1 demonstrates
parameterized expiry without another full200 replay. Source event/radius is
the same T200 event for all R4 window comparisons.

Remaining full service mean/median/P95/max ms: R3
24.6762/28.5989/32.6185/36.7745;20/20
17.5343/19.7326/21.3908/22.7259;40/40
28.8110/34.8133/41.8752/43.8457;30/10
10.8776/11.4622/13.3694/14.5909;outside
22.6952/27.4650/32.0249/32.7733. Main HPL maxima
4.55790/5.06488/5.06510m, VPL maxima7.33206/7.37146/7.37157m
(R0/UA/UG). Corresponding protection-disk diameters reach10.13020m,
vertical interval widths14.74315m; these are conservative position bounds.
Finite rate952/952 admitted outputs. AL failures do not negate the theorem.

### Resources, access and timing limits

| Quantity | Main30/30 maximum | Other tested maximum/scope |
|---|---|---|
| Live FGO states/values |30/90|40/120 in40/40; no lag enlargement|
| Native intervals/raw endpoint copies |29/319|39/429 in40/40; repeated endpoints are not new noise|
| Proof explicit batches after projection |29|19/39/9 for20/20,40/40,30/10|
| Observed IMU replay ring |501|401 follow20;601 follow40/delayed;301 outside|
| Raw recovery capsules |51|61 follow40/delayed;41 follow20;31 outside|
| Set/capsule/material payload |121960 bytes|161160 follow40;125400 delayed;50440 explicit10|
| Stored IMU physical parameters |2400|12T fixed task bank; UWB16T symbolic recipe|
| Joint logical hypotheses |7685601|0 dense joint optimizers/inventories|
| Native ledger entries |141|178 delayed|
| Native factor slots |253|empty tombstones included, see below|
| Resident memory after service |23848KiB|24424KiB follow40; provider included|
| Separate simulation provider |2001 IMU/200 batches|full requested playback; no retired-prefix online reads|

At200 proof reads UWB171–200 and new IMU endpoint IDs1991–2001, then
retains separator171 and29 explicit batches. Recovery uses live graph keys
and bounded capsules, checking the preceding shared endpoint. It never scans
epoch0..k or reads the provider's retired raw prefix. `raw_read_first/last`
denote material endpoint ownership; `imu_read_first/last` and
`proof_uwb_first/last` denote actual sensor reads. Actual keys, marginalized
keys, numerical boundary/removed-prior slots, method/scope/frontier/event are
verified in the transient trace. Initial p/v/R raw is unavailable to the proof.

Explicit state/raw/proof storage is bounded by Lf+Li+replay. GTSAM's native
slot array retains empty tombstones and grows roughly one slot per marginal
epoch: R0 max234, UA253, UG235 at200. This is disclosed, not claimed as O(1)
total process memory; tombstones hold no retired raw information and are not
a full-prefix rebuild. Task cap10000 and the O(T) IMU parameter recipe are
explicit finite limits. Open-ended slot compaction, time-uniform risk,
repeated episodes and long-run box tightness remain outside this completion.

Platform: WSL2 Linux6.18.54.1 x86_64, Intel Core Ultra5 125H, TBB/OMP/BLAS/MKL
limited to1. Core includes ingest, native preparation, complete action bank,
temporary candidate GN, proposal, one commit, state/receipt/ledger checks,
raw issue, cleanup and resource reads. Full adds serialized output packaging.
CSV I/O is separate: main totals15.8042/15.7942/15.6650ms,
maxima .1654/.2255/.1962ms; other-row maximum .8857ms. Cold source
initialization exceeds50ms. These are PC continuous service samples, not
hardware arrival/transport/TTA, paced-queue qualification, P99 or WCET.
**ROLLING_PERFORMANCE_OPEN** remains.

Final related safety regression:10 invocations PASS — `sc5_math_checks`,
`sc4_system_checks`, its `carry` fixture, `sc3_joint_checks`, `sc2_imu_checks`,
`raw_range_checks`, `hybrid_raw_checks`, `hybrid_checks config/fde_next.yaml`,
`epoch_raim_checks config/fde_next.yaml`, `fused_epoch_checks config/fde_next.yaml`.
This is one core regression, not SC1–SC4 campaigns. It covers original50/6421,
expiry/carry, joint response/cancellation, frontier/binding, retained covariance
and legacy recovery/receipts. The final SO(3) witness retains p=(-1,0,0),R=I,
lever=(1,0,0),tag=0; a small tag cannot reset body/rotation uncertainty.

### RI01–RI16 audit

| Clause | Authoritative evidence | Result |
|---|---|---|
| RI01 |clean d668bcb entry; old configs/source/default/release unchanged; final legacy checks|PASS|
| RI02 |four real profiles;560 observation/control commits; invalid/hot resize refusals; independent T event|PASS|
| RI03 |actual retained/marginal keys and source receipts; x0 exits; requested Lf unchanged|PASS|
| RI04 |dense SVD residual/dof/state/joint two-column Gram/cross/RHS/response; shared latent prior|PASS|
| RI05 |directed p/v/R/bias reach/inverse/projection;100-digit oracle; operator inclusion proof|PASS|
| RI06 |tag=p+Rl inverse/SO(3) witness; p→v→future p; ended-fault→future PL; no state reset|PASS|
| RI07 |actual parent numerical set consumed; bounded raw released;200 streaming outputs; no t0 formula|PASS|
| RI08 |unique ADC/RW/shared endpoint; persistent shared-theta bank and residual phases; one event|PASS|
| RI09 |N(T), integer/binary tail proof,11 T sizes, adaptive task induction and T+1 refusals|PASS|
| RI10 |three T200 streams600/600,51/90/150/200 qualified; final953 admissions reconciled|PASS|
| RI11 |UA/UG isolation61 after x0 exit; actual bridge and CombinedIMU72–75; single mutation|PASS|
| RI12 |marginalized onset20 pollution repaired55 by real prior replacement; replay60 positive/replay30 refusal|PASS|
| RI13 |stale/future/batch/scope/set tamper, rank/resource/wrong-FDE/ledger, AL, pre/post-update and retry tests|PASS|
| RI14 |actual states/raw/capsules/modes/ledger/slots/resident/provider/read ranges; compact product; finite growth disclosed|PASS finite scope|
| RI15 |complete service/late/cold/I/O denominators;10 final regressions; no hardware/P99/WCET claim|PASS evaluation; performance OPEN|
| RI16 |necessary source/test/config, one report, local W0–W6 saves, reproduction and own transient cleanup|PASS after W6 save|

### Reproduction and final scope

From the repository root in its existing ROS/GTSAM build environment:

```bash
cmake -S . -B /home/mint/ws_fusion_uwb/build/uwb_imu_pl
python3 tools/sc4_system_research.py rolling-check --build
python3 tools/sc4_system_research.py rolling
```

The matrix command removes its own temporary traces. `--main-only` and
`--remaining-only` permit targeted resumption. The existing native runner is
`epoch_raim_runner CONFIG CSV SCENE rolling on|off PLAYED [FDE_ENABLE_EPOCH]`.
Python supplies the explicit local library path. Development logs/CSV/configs
are removed after reconciliation; no raw/binary/evidence backup is retained,
no old/user files are deleted, no automatic push occurs.

Final: **SC5_FINITE_ROLLING_INTEGRITY_COMPLETE**,
**ROLLING_RAW_BOUND_VERIFIED_FOR_FINITE_SCOPE** and
**ROLLING_FDE_VERIFIED_WITHIN_RECOVERY_HORIZON**. This is conditional on the
declared simulation source/physical/fault model. **ROLLING_PERFORMANCE_OPEN /
TIME_UNIFORM_OPEN / APPLICATION_AL_UNQUALIFIED / DEPLOYMENT_UNQUALIFIED**
remain. Recovery outside the buffer is unavailable; actual navigation recovery
for other axes and full15D accuracy are not inferred from accel-x/gyro-x.
No infinite mission, renewed episode budget, real-data calibration or formal
release is claimed.

## Goal02 — open-ended development contract

Entry actual clean HEAD ea31992. RI01–RI16 source and native evidence above
are the prerequisite; no mock upstream qualification is used. Production and
finite mission entry stay unchanged. New research scope permits repeated
non-overlapping episodes per channel: at most one active UWB single-anchor
episode (1/3 epochs) and one active IMU single-axis episode (50/500ms), with
all six IMU axes. UWB and IMU may coexist. Unknown/out-of-scope concurrency
is not assigned zero risk. Physical motion/noise/extrinsic conditions remain
the original registered simulation conditions. No hardware/AL qualification.

Frozen positive inputs before results: healthy500; mixed1000 with UA61 and
UG321; repeated UG500 with UG61 and UG261. Original amplitudes20m/s²,
1rad/s,2.25m and seed20261010; no seed search. Short window variants20/20,
40/40,30/10 bind the same infinite event. Negatives include mixed certificate
modes, source reuse, future changes, capacity, concurrency, cancel/overload,
and out-of-replay recovery. Long-flow samples prove runtime dependencies,
not the all-time theorem. Core is OPEN until boundary/repeated recovery runs.

Goal02 event implementation: directed allocation lower/upper endpoints;
integer q with5q²≥7m and exact binary2n·2^-m≤delta_lower, n0=21,nk=128.
The infinite telescope is eps/2+sum eps/(2k(k+1))=eps, without independent
windows.100-digit oracle PASS1001 blocks plus large global indices;
60006 unique ADC IDs, shared sample10 retains block1 at reuse in epoch2.
q0/200/1000=7/8/9. q has no ceiling; representation failures refuse rather
than restart or truncate. First compilation failed on a multiprecision
conditional expression type; explicit assignment repaired it,0 native epochs.
Risk accounting alone is not upstream/time-uniform rolling qualification.
