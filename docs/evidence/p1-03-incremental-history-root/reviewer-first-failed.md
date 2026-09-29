# P1-03 first independent lifecycle review: FAILED

The first independent reviewer accepted the arithmetic history root, all
correctness gates, the three strict 45-attempt comparisons, ABI clients and
the bounded-RSS observation. It rejected acceptance for four lifecycle and
archive gaps:

1. scoped commit tokens were globally retained and were not revoked by arena
   close; close and consume had no shared linearization lock;
2. committed-state publication sidecars remained in an append-only global map;
3. a non-empty scoped final packet silently fell back to the legacy registry
   if final-bundle freezing failed;
4. the Section 2.5 archive was incomplete.

No commit, tag or P1-04 transition was authorized by that review. The repair
is limited to those four findings and is covered by the subsequent 7/7 P103
lifecycle tests, P0-03/P0-05 reruns, complete CTest, Section 2.4 comparison
and 120-attempt soak recorded in this directory.

