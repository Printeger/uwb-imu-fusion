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
