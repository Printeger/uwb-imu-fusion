# P0-05 second independent review — FAILED

P0-05 final independent review was **FAILED**, not `BLOCKED`.  The reviewer
performed a read-only review and did not modify the worktree.

1. Normal clean/full-build/tests/CTest: **PASS**.  The round-2 runner was
   present and the complete result was 28/28.
2. P0-03 sidecar and pre-mutation rejection: **PASS**.  Fake, tampered and
   replayed evidence was rejected before mutation, and the legacy path was
   explicitly unprotected.
3. O08 complete-state snapshot: **FAILED**.  Ledger, catalog, history, IMU and
   health checks compared mainly counts or sizes.  The evidence did not compare
   complete ledger lifecycle and IDs, prune identities, the full navigation
   state (including attitude and biases), IMU contents/cursor, or complete
   `HealthRecord` contents at every injection point.
4. O04 actual backend/reference oracle: **FAILED**.  The alleged genuine test
   only minted and consumed a token and did not perform a certified commit.
   The large-delta triangle used an artificial `large_committed` value, while
   actual backend state and covariance were checked only on the legacy
   unprotected path.
5. Final packet and publication boundary: **FAILED**.  The callback consumed an
   invocation packet and the function rebuilt a different terminal packet after
   callback return.  The independent probe produced callback digest
   `12090195748681269483` and returned digest `17629356177082681612`
   (`equal=0`).  With arrival=100, call=140, return=1170 and deadline=100, the
   result incorrectly remained `deadline_missed=0`, protected.  A partial
   publisher exception could publish an earlier protected ROS sink before a
   later journal consumed a rebuilt unprotected packet.
6. 90-degree yaw, `R^T` velocity and non-diagonal covariance: **PASS**.
7. ROS raw evidence reproducibility: **FAILED**.  Only a text capture from one
   run was retained; the claimed capture program was absent.  The captured
   attempt was unavailable/rejected and did not exercise a certified committed
   reference transfer.
8. ABI/evidence/regressions: **FAILED**.  P0-01 through P0-04 regressions,
   hashes and explicit `NOT_RUN` records passed, but public
   `ProtectionLevelPublicationPacketV1` changed from golden size/offset
   `88/80` to candidate `256/248`, and the ABI probe omitted that type.  V1 must
   remain unchanged and the new proof must use a V2 sidecar/API.

The reviewer therefore prohibited updating the master table, committing, or
creating `golden-p0-05-publication` until these P0-05-only findings were
repaired.
