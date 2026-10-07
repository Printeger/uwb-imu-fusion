# Off-profile position/attitude quality diagnosis

The final-source 20-seed campaign passes position RMSE, position P95 and
velocity RMSE for 20/20 seeds, but passes the configured 0.1-rad attitude RMSE
gate for only 4/20.  The final-source 3x12000 campaign passes position RMSE for
2/3, position P95 for 3/3, velocity RMSE for 3/3, and attitude RMSE for 0/3.

Checks against the raw records found aligned truth/state timestamps,
near-unit quaternions, and agreement between the independent geodesic
calculation and the recorded metric.  Across the 20 seeds the truth-local
rotation-vector RMS by axis is approximately 0.01253, 0.01671 and 0.16463 rad;
the third component dominates.  Position RMSE and attitude RMSE have
correlation 0.999913.  Their mean slope is about 0.2684 m/rad, close to the
configured lever-arm norm `sqrt(0.25^2 + 0.10^2 + 0.08^2) = 0.2809 m`.

For a single UWB antenna the observation is the antenna position `p + R*l`.
Each range rotational Jacobian is `u_i^T R[-l]_x`; these rows share the right
null direction `l`, so the rotational block has rank at most two.  Rotation
about the lever axis can be paired with translation of the body origin while
preserving antenna ranges.  This explains the measured attitude/body-origin
coupling and is not a denominator, frame, quaternion, timestamp or scoring
bug.

There is no permitted implementation-only repair.  Adding another antenna or
sensor, or changing the model, prior or noise would expand the authorized
scope.  The quality gates therefore remain `FAIL`; no threshold, sample count
or denominator was changed.
