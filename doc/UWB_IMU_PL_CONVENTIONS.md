# UWB–IMU–PL conventions

Status: `IMPLEMENTED_UNVERIFIED`.

- Time is signed 64-bit nanoseconds. Conversion from ROS seconds occurs once at the adapter boundary; replay ordering uses `(timestamp_ns, sequence)`.
- The world frame is right-handed, Z-up. A pose maps body-frame points into world: `p_W = R_WB p_B + t_WB`. Quaternions are stored as `q_world_body` and Eigen uses `(w,x,y,z)` at serialization.
- IMU acceleration is specific force in the body frame in m/s²; angular velocity is body-frame rad/s. Gravity in world is `(0,0,-g)`.
- UWB measurements are TWR ranges in metres. Anchor positions and protected position are in the configured world frame. The tag lever arm is expressed in the IMU/body frame.
- Snapshot residual is `z - h(x)`. Both Jacobian and residual are whitened by the same lower-Cholesky inverse of the configured covariance.
- GTSAM `Pose3` tangent ordering is rotation then local translation. World-position extraction therefore uses `[0, R_WB]`; covariance indices are never assumed to be world XYZ directly.
- Every stochastic run requires an explicit seed. Every formal output is bound to config hash, git SHA, graph/ordering/noise/linearization version and factor/measurement provenance.

The initial release supports 3D TWR only. TDoA, multiple simultaneous faults, persistent historical-fault PL, FDE and fixed-lag provenance preservation are explicitly out of scope.
