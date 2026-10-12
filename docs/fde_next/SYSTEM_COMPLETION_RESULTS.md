# System completion — SC1

Status: implementation in progress; no SC1 acceptance claimed.
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
