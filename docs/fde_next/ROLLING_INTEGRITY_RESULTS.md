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
