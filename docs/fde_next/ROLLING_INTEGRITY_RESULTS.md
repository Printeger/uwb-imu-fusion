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
