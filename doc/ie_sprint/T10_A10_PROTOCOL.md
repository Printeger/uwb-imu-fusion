# T10-A10 generation and development preregistration

Registered 2026-09-09 before generator implementation and before any A10 estimator process.
Authorization: this user/commander session accepts
`REVIEW_ACCEPTED_A09_BOUNDED_STAGE2_BUDGET_SCOPE`; no external REVIEW.md is asserted.
T10 remains IN_PROGRESS. A08 graph FD remains 15/18; three smallest-step failures remain UNKNOWN.

## Scope, assumptions and data roles

Only development is executed. No old bag is used. Old T07 total latent base bias stays UNKNOWN.
Reuse `LoadT07ScenarioCache`, shared initializer, GraphBuilder and C++/GTSAM automatic discovery,
refit and scoring. Add a Python **generator**, consistency checker and bounded orchestration,
not an estimator. No solver/default/stop/tolerance/group/short/boundary changes.
Known geometry, beta and noise below are exact **synthetic assumptions**, not measured or
independently calibrated real hardware. Evaluation truth is forbidden estimator input.

World is right-handed z-up; R maps IMU body to world; positions/metres, time/seconds,
velocity m/s, acceleration and accelerometer specific force m/s², angular rate rad/s.
g_world=(0,0,-9.81). Body origin is antenna origin: lever=(0,0,0) m, explicitly co-located.
One tag id=0; eight fixed anchors are the Cartesian product x,y={-3,3} m, z={0,3} m,
ordered x then y then z, anchor ids 1..8. Fixed beta by link is
[0.03,-0.02,0.04,-0.01,0.02,-0.03,0.01,-0.04] m. No anchor uncertainty,
clock offset/drift, dropouts, random NLOS or other hidden range error is simulated.
Every range satisfies z=norm(p+R*lever-a)+beta+b_total+epsilon.
`b_total` is the complete nonnegative dynamic latent excess path, excluding signed fixed beta
and zero-mean noise; also export beta+b_total as total deterministic range offset.

## Shared analytic motion and samples

Full recording t=0..8 s. IMU at j/200 s (1601 samples); simultaneous eight-anchor UWB
messages at k/5 s (41 messages, 328 observations). Sensor time origin exactly 0;
no rounding, jitter, asynchronous association or future interpolation. All UWB messages
are keyframes (step=1), nominal range sigma=.05 m. `v_max=0` disables the existing
extra time-gap variance because ranges are exactly at keyframes; it is not a bound on true speed.

Static until t0=2.2 s. For u=max(0,t-t0), tau=.8 s and rate=.45 rad/s:
theta=rate*[u-2*tau*(1-exp(-u/tau))+tau/2*(1-exp(-2*u/tau))];
theta_dot=rate*(1-exp(-u/tau))²;
theta_ddot=2*rate*(1-exp(-u/tau))*exp(-u/tau)/tau.
p=(.8*sin(theta), .6*(1-cos(theta)), 1+.1*(1-cos(2*theta))).
Velocity and acceleration are analytic first/second derivatives.
yaw=.4*sin(theta), R=Rz(yaw), omega_body=(0,0,.4*cos(theta)*theta_dot).
Specific force f=R^T*(p_ddot-g_world). This is a modest turning/height-varying trajectory,
not a full 3D excitation study. Both sensors use exactly these functions.

Constant IMU latent biases: ba=(.003,-.002,.004) m/s², bg=(.0002,-.0001,.00015) rad/s.
No random walk is generated. Raw IMU=f+ba+epsilon_a, omega+bg+epsilon_g.
Accelerometer/gyro independent white Gaussian per-sample std are
.002/sqrt(.005) m/s² and .0002/sqrt(.005) rad/s.
The estimator density parameters are .002 and .0002; existing nonzero estimator bias random-walk
model (.01 m/s²/sqrt(s), 2e-5 rad/s/sqrt(s)) remains an explicit model assumption,
while generated biases are constant. No truth orientation in observations: has_orientation=0,
identity quaternion placeholder; use_imu_orientation_init=false. Initial pose is computed
from raw gravity/ranges by existing initializer, velocity initially zero; biases not supplied.
Existing initialization-derived pose/velocity/bias priors remain heuristic initialization priors,
not independent measurements (sigmas .01 rad/.05 m, .1 m/s, .01 bias mixed native units).

RNG: NumPy Generator(PCG64(SeedSequence([seed,stream_id]))), independent streams
0=UWB, 1=accelerometer, 2=gyro. Draw shapes fixed in sample-major/component-major order.
Archive NumPy/Python versions and generator SHA. Same seed promises byte reproducibility
in the recorded environment, not across unspecified NumPy versions. Seed 10101 is used
for all three paired development scenarios (one actual random seed per scenario).
Seed 10102 is only a generator metamorphic check; no estimator run on it.

LOS: b_total=0 everywhere. Step: links 0:1/0:2, closed interval [3,6] s,
b_total=.6/.9 m respectively. Ramp: same links/interval,
b_total=.2+.2*(t-3) / .3+.25*(t-3) m. Outside interval b_total=0.
No change to these truths after any estimator output. Paired scenarios share raw noise and motion.

## Serialization and isolation

Write existing exact `t07_estimator_cache_v1` CSV/schema/canonical identity; recording ID
binds generator version, base motion definition and seed, obs_id uses existing FNV1a64
recording/message/range convention and remains stable across scenarios. Cache ID additionally
binds actual noisy IMU/UWB bytes. Generator manifest separately binds source/parameters/
role/seed/truth/cache hashes. Raw directory contains only input manifest and two sensor CSVs.
Separate evaluation directory contains poses/kinematics, IMU latent biases and draws, UWB
geometry/beta/total latent bias/draws keyed by obs_id. No truth/support/GT path in config.
Estimator file opens will be traced; evaluation directory is unavailable at its documented path
during all pilot processes. Preserve trace, actual library maps/identity and before/after hashes.

## Fixed support candidates and finite budget

All other effective fields inherit the accepted A09 isolated config. Only new sensor/input
assumptions above and these preregistered support values differ. `T_gap=1 s`, short count=2,
short duration=.01 s, boundary=1e-9 m and group/merge algorithm are unchanged.

| id | lambda_l1 (1/m) | lambda_tv (1/m) | b_min (m) | change (m) | merge (m) |
|---|---:|---:|---:|---:|---:|
| P1 | 4 | 20 | .10 | .15 | .10 |
| P2 | 8 | 40 | .15 | .20 | .15 |
| P3 | 12 | 80 | .20 | .25 | .20 |

At sigma=.05, w=400/m², isolated L1 shrinkage lambda/w=.01/.02/.03 m.
For 16 samples a constant plateau's two-edge TV shrinkage is about
2*lambda_tv/(16*w)=.00625/.0125/.025 m. Activity thresholds are 2/3/4 nominal
range sigmas; change 3/4/5 sigmas and merge 2/3/4 sigmas express noise-scale support
hypotheses, not a promise of eligibility. All candidates compare identical three inputs.

Run order P1 LOS/step/ramp, P2 LOS/step/ramp, P3 LOS/step/ramp. Exactly 9 planned estimator
processes, hard 120 s each (SIGKILL at deadline), no retry/result-driven expansion.
Stage1 cap500/V2, refit cap200, inner50; original tolerances and stopping unchanged.
No gate/final/held-out runs. Maximum scheduled estimator time 1080 s;
development B_total=1440 s with 360 s (25%) unallocated reserve, not permission to retry.
Generator/checker work is separately timed, not counted as estimator trials.
All failure/timeouts/zero-eligible rows stay. Valid scoring triggers independent F/G/N/R/gamma
provenance/recomputation checks; unavailable scores stay unavailable.

## Before pilot acceptance checks

Analytic v/a vs central differences; R^T R=I/det=1 and R-dot/omega consistency;
noise-free inertial forward integration at dt=.005 vs analytic pose/velocity with declared
error limits .003 m/.003 m/s; unit/time endpoints and strict order; measurement equation
closure <=1e-12 m, inertial closure <=1e-12 native units; same-seed byte equality;
different seed changes actual UWB and both IMU random channels while preserving motion;
raw orientation absent; full one-to-one obs_id truth coverage; cache hashes/canonical ID;
negative measurement/truth corruption must fail. These test thresholds are engineering
checks, not changes to estimator tolerances. Pilot will also validate C++ loader identities,
nominal sigma and raw range retention by joining actual ledgers back to inputs.
