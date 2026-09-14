# REFACTOR-GATE-06R-D: frontier-51 initialization failure diagnosis

## Scope and verdict

This was a diagnostic-only replay of the frozen SFUISE Walk1 `state_step=1`
input. It did not read GT, truth, oracle data, or ATE; it did not run Gate06,
the final estimator, or any IE stage. The production initializer, retry count,
stationarity limit (`1e-5`), Huber scale (`1.345`), range sigma (`0.15 m`), LM
settings, physical factors, repeat representatives, and ledger were unchanged.

The primary classification is
`A. ONE_STATE_FRONTIER_CONDITIONING_FAILURE`, more precisely a
**fixed-history one-state conditional-formulation failure, not a numerical
rank deficiency**. The state-51 one-state Jacobian is full rank with a modest
condition estimate, but the frozen-boundary problem reaches a repeatable
IMU/UWB cancellation floor just above the qualification limit. An additional
identical continuation cannot move it. Letting only the three states after the
fixed causal 0.25 s boundary move jointly satisfies the same qualification.

## Reproduction and evidence boundary

The final isolated evidence is
`experiments/icra2027/dev/COMMON_INITIALIZATION_FRONTIER51_DIAGNOSIS/run3/`.
Its `summary.json` and `frontier51_conditioning.json` both pass
`python3 -m json.tool`. The run reports
`gt_truth_oracle_read=false`, `gate06_final_run=false`, and
`ie_stages_run=false`. `command.txt` records the exact invocation. The
diagnostic executable was built as the dedicated
`uwb_imu_fgo_frontier51_diagnosis` target and exited 0.

The diagnostic loaded 913 physical states, 2,276 physical UWB factors, and
3,191 physical factors in total. It then reproduced the production causal
frontier sequence rather than optimizing the full graph.

## Accepted-frontier trend, states 45--50

Bias columns are `[bax,bay,baz] m/s^2 / [bgx,bgy,bgz] rad/s`. Position and
velocity vectors are in metres and metres/second. All numbers are computed
without GT.

|k|timestamp (s)|position|`|p|`|velocity|`|v|`|bias accel / gyro|
|---:|---:|---|---:|---|---:|---|
|45|1664959679.9463379|[15.189510, 0.098296, -6.995404]|16.723234|[11.774133, 1.950038, -16.324554]|20.221867|[7.891e-6,-1.713e-5,5.091e-5] / [-0.00717969,-0.00750019,-0.00491301]|
|46|1664959679.9976201|[15.792044, 0.200849, -7.858270]|17.640334|[11.724475, 2.049561, -17.327038]|21.021186|[7.891e-6,-1.713e-5,5.091e-5] / [-0.00717969,-0.00750019,-0.00491301]|
|47|1664959680.0723972|[16.660386, 0.359362, -9.204935]|19.037553|[11.504549, 2.190938, -18.703158]|22.067238|[4.614e-6,-1.655e-5,5.224e-5] / [-0.00718003,-0.00750156,-0.00491324]|
|48|1664959680.1231875|[17.241213, 0.473168, -10.179568]|20.027654|[11.368083, 2.291083, -19.675742]|22.838940|[4.451e-6,-1.652e-5,5.229e-5] / [-0.00718005,-0.00750164,-0.00491325]|
|49|1664959680.1985250|[18.086000, 0.651455, -11.713137]|21.557490|[11.062943, 2.441503, -21.046358]|23.901858|[1.744e-6,-1.607e-5,5.315e-5] / [-0.00718028,-0.00750254,-0.00491347]|
|50|1664959680.2491667|[18.641291, 0.777624, -12.803003]|22.627843|[10.867053, 2.541198, -21.996352]|24.665564|[1.599e-6,-1.604e-5,5.320e-5] / [-0.00718030,-0.00750265,-0.00491348]|

The residual entries below are signed mean/median and maximum absolute value.
Standardized residuals use the unchanged 0.15 m sigma. Huber weights are
mean/min/median/max.

|k|UWB / anchors|raw residual m|standardized residual|Huber weights|objective initial -> terminal|max gradient|update norm|LM calls / accepted / rejected; lambda|
|---:|---|---|---|---|---|---:|---:|---|
|45|5 / 7475,9524,10548,15155,20276|10.9640 / 10.3879 / 13.4859|73.0935 / 69.2530 / 89.9063|0.12763 / 0.09973 / 0.12948 / 0.16642|487.807132 -> 487.387062|5.669e-6|0.260260|15 / 15 / 0; 1e-5 -> 1e-20|
|46|0 / none|N/A|N/A|N/A|9.413e-34 -> 9.413e-34|8.144e-16|9.803e-18|0 / 0 / 0; diagnostic trace 1e-5 -> 1e5|
|47|5 / 7475,9524,10548,15155,20276|13.4165 / 12.6982 / 16.0244|89.4434 / 84.6546 / 106.8296|0.10279 / 0.08393 / 0.10592 / 0.12680|597.606806 -> 597.271182|9.609e-6|0.232990|15 / 15 / 0; 1e-5 -> 1e-20|
|48|1 / 7475|16.6979 / 16.6979 / 16.6979|111.3196 / 111.3196 / 111.3196|0.08055 all|148.826902 -> 148.823600|6.859e-7|0.018993|5 / 5 / 0; 1e-5 -> 1e-10|
|49|4 / 7475,9524,10548,20276|15.4183 / 15.0689 / 18.2163|102.7889 / 100.4596 / 121.4418|0.08834 / 0.07384 / 0.08928 / 0.10098|549.721678 -> 549.544427|3.537e-6|0.171657|13 / 13 / 13; 1e-5 -> 1e-5|
|50|1 / 20276|14.4773 / 14.4773 / 14.4773|96.5150 / 96.5150 / 96.5150|0.09290 all|128.917765 -> 128.912911|7.519e-7|0.023203|5 / 5 / 0; 1e-5 -> 1e-10|

Every row terminated
`CONDITIONAL_LM_CONVERGED_AND_NAVIGATION_STATIONARY`, except state 46, which
was already stationary. Position/velocity norms continue smoothly across
these six frontiers. Residual magnitude is large but does not create a
monotonic solver-failure trend: gradients alternate from `9.609e-6` down to
`6.859e-7`, up to `3.537e-6`, then down to `7.519e-7`. Thus frontier 51 is an
abrupt qualification event, not evidenced `ONGOING_INITIALIZATION_DRIFT`.
The complete-precision table is `accepted_frontiers_45_50.csv`.

## Frontier 51 local audit

State 51 is at timestamp `1664959680.3226264 s`. Its graph has six factors:
one fixed-boundary physical `CombinedImuFactor` equivalent and five physical
range `ExpressionFactor<double>` clones with only the initialization Huber
noise wrapper. Its five anchors are 7475, 20276, 9524, 10548, and 15155.

Before optimization, the IMU prediction was:

- quaternion xyzw `[0.1407616331, 0.9624254497, 0.09094594575, -0.2136638749]`;
- position `[19.42927568, 0.969591293, -14.46969808] m`, norm `24.24477311 m`;
- velocity `[10.58580955, 2.685541919, -23.38003144] m/s`, norm `25.80498730 m/s`;
- accelerometer bias `[1.599200e-6, -1.603685e-5, 5.320417e-5] m/s^2`;
- gyro bias `[-0.007180303, -0.007502649, -0.004913484] rad/s`.

The predicted displacement from accepted state 50 is `1.853549502 m` over
`0.0734597 s`, consistent in scale and direction with the adjacent velocity;
the prediction is finite and physically local relative to state 50. This is a
local continuity statement only, not a GT accuracy claim.

|anchor|measured m|predicted before m|residual before m|q before|weight before|residual terminal m|q terminal|weight terminal|
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|7475|3.210000|24.144558|20.934558|139.5637|0.06425|20.926514|139.5101|0.06427|
|20276|3.790000|20.013217|16.223217|108.1548|0.08291|16.213380|108.0892|0.08296|
|9524|4.130000|22.153971|18.023971|120.1598|0.07462|18.015341|120.1023|0.07466|
|10548|3.560000|21.377944|17.817944|118.7863|0.07549|17.807520|118.7168|0.07553|
|15155|1.640000|23.250462|21.610462|144.0697|0.06224|21.600751|144.0050|0.06227|

Before-solve signed residual mean/median/max-absolute are
`18.9220/18.0240/21.6105 m`; standardized values are
`126.1469/120.1598/144.0697`; weight mean/min/median/max are
`0.071900/0.062238/0.074623/0.082906`. These magnitudes extend the already
observed robustly downweighted local regime. No one anchor dominates: all
five residuals have the same sign, span only 16.22--21.61 m, and their weights
span 0.062--0.083.

The first checked-LM block reduces objective `843.8151331305` to
`843.5933278112` with update norm `0.1869949575`. It accepts 15 updates in 16
calls, has 25 rejected lambda trials, drives lambda from `1e-5` down through
`1e-20`, and the no-update return then exhausts at approximately `1e5`.
Termination is `CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED`. The terminal state is:

- quaternion xyzw `[0.1389777363, 0.9500242366, 0.08444463895, -0.2664737167]`;
- position `[19.42476414, 0.969368349, -14.46570390] m`, norm `24.23876494 m`;
- velocity `[10.46992425, 2.680382279, -23.28246484] m/s`, norm `25.66859817 m/s`;
- accelerometer bias `[-1.529755e-6, -1.536566e-5, 5.400517e-5] m/s^2`;
- gyro bias `[-0.007180523, -0.007503534, -0.004913599] rad/s`.

Its maximum scaled navigation gradient is `1.0741484355758502e-5`, only
`7.4148%` above the frozen limit. The sole dominant component is `x51`
coordinate 5, pose translation z. The currently permitted continuation has
one call, no accepted update, ten rejected lambda trials, objective unchanged,
state-update norm `2.1984e-17`, lambda `1e-5 -> 1e5`, the same termination,
and the same dominant gradient. Its X/V/B and all five UWB residuals are
bit-for-reported-digit unchanged from the first terminal state.

## One-state linearization and conditioning

At that exact failed terminal Values, the whitened Jacobian is `20 x 15` and
has rank 15. The `15 x 15` normal matrix also has rank 15. The Jacobian
singular spectrum, descending, is:

`[204.9760911, 204.9760911, 204.9760906, 166.7013680, 166.1011282,
166.0946456, 83.85390094, 83.85389823, 83.85389823, 4.118391186,
4.095084643, 3.626522988, 3.458373422, 3.016077142, 3.010883223]`.

The normal eigen spectrum, ascending, is:

`[9.065417783, 9.096721328, 11.96034673, 13.15166898, 16.76971823,
16.96114596, 7031.476248, 7031.476248, 7031.476703, 27587.43129,
27589.58478, 27789.34608, 42015.19770, 42015.19793, 42015.19793]`.

The smallest/largest meaningful singular values are `3.010883223` and
`204.9760911`, giving condition estimate `68.0784`; normal extrema are
`9.065417783` and `42015.19793`. Therefore this is not rank loss or an extreme
linear numerical condition number.

The weakest right-singular direction is primarily `v.z=+0.6621`,
`x.rot_y=+0.5635`, and `v.x=-0.4757`, with smaller `x.rot_x=+0.1194` and
`v.y=-0.04734`. Bias coefficients are at most `2.92e-5` in that direction.
The IMU/fixed-boundary gradient has L2 norm `44.59021501`; the UWB gradient has
L2 norm `44.59021499`; they oppose almost exactly. Their total has L2 norm
`1.1232682e-5`, dominated by translation z at
`-1.0741484e-5` (IMU `-10.83552973`, UWB `+10.83551899`). Bias-coordinate
gradient terms are only about `1e-14`, so IMU bias is not the stationarity
failure. Exact per-coordinate values are in `frontier51_conditioning.json`.

## Identical same-graph continuation

One additional diagnostic-only continuation started from the exact failed
terminal state with the identical graph, Huber, sigma, LM settings, lambda
limits, and `1e-5` test. It made no accepted update, rejected all ten lambda
trials from `1e-5` through the upper limit, and returned
`CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED`. Objective, X/V/B, residuals, dominant
translation-z component, and maximum gradient
`1.0741484355758502e-5` remained unchanged; update norm was `2.1984e-17`.

The gradient therefore remains at the same numerical floor and does **not**
cross the condition. This excludes `MARGINAL_QUALIFICATION_CLIFF` under the
task's rule and supplies no evidence for increasing the production retry
budget.

## Fixed 0.25 s causal joint-window replay

The earliest in-horizon state is state 48 at `1664959680.1231875 s` and is
fixed as the boundary. States 49, 50, and 51 are optimized jointly. The exact
span is `0.1994388103 s`; only factors ending at those states and observations
with timestamps no later than state 51 enter. The graph has 13 factors: three
physical Combined IMU factors and ten UWB factors (4 at state 49, 1 at state
50, 5 at state 51). No future measurement enters.

The initial objective is `1522.272471422`; terminal objective is
`1517.522899654`. The unchanged solver returns
`CONDITIONAL_LM_CONVERGED_AND_NAVIGATION_STATIONARY` after 45 calls/accepted
updates and 50 rejected trials, with lambda `1e-5 -> 1`. Maximum scaled
navigation gradient is `4.905753878e-6`, below the same `1e-5` condition.
The joint Values update norm is `2.292012487`; state 51 update norm is
`1.691778981`. Every X/V/B state in the free window is finite.

At the window terminal, state-51 residual mean/median/max-absolute are
`18.7352/17.8403/21.4223 m`; standardized values are
`124.9016/118.9351/142.8150`; Huber weight mean/min/median/max are
`0.072635/0.062785/0.075391/0.083885`. The window succeeds without removing an
anchor or making the already downweighted UWB residual regime qualitatively
different.

## Causal attribution and repair boundary

The accepted history is locally continuous, state 51 is a finite and
kinematically local IMU prediction, no anchor is singularly pathological, and
the accepted-frontier solver metrics do not show systematic deterioration.
The failed local matrix is full rank, but freezing every historical state
forces the frontier state alone to balance two large opposing IMU and UWB
gradients; it settles at a deterministic translation-z cancellation remainder
just above the certificate boundary. Repeating that same conditional problem
does nothing. Allowing only the recent causal states to redistribute the same
factor conflict lowers the gradient below the unchanged qualification.

These observations satisfy the task's definition of
`ONE_STATE_FRONTIER_CONDITIONING_FAILURE`. “Conditioning” here denotes the
over-restrictive one-state conditional problem relative to the causal joint
problem; it must not be reported as Jacobian rank deficiency.

The narrowest justified production repair is to retain the current one-state
fast path and existing single identical continuation, but on its fail-closed
result construct the already-fixed 0.25 s causal window, fix its earliest
state, jointly optimize only subsequent X/V/B using the identical factors and
settings, and atomically commit the whole free window only if all existing
finite/key/objective/stationarity checks pass. Otherwise it must still return
empty Values. This recommendation does not authorize a retry increase,
threshold/kernel/sigma/LM change, future data, or any final-estimator/IE
change. It was not implemented in this task.

GATE06R_D_ROOT_CAUSE_IDENTIFIED
