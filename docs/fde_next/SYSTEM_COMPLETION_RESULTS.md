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
