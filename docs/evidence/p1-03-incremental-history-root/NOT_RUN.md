# P1-03 explicit NOT_RUN

- TSan/MSan/ASan: NOT_RUN; the configured Release acceptance build does not
  provide sanitizer targets.  Concurrent lease readers, exception cleanup and
  full CTest were run.
- Physical UWB/IMU hardware: NOT_RUN; P1-03 is an internal history/proof
  lifetime optimization and uses the frozen deterministic benchmark.
- Formal deployment qualification: NOT_RUN and unchanged.  Protected/formal
  output remains behind the existing fail-closed qualification gate.
- Tail-latency improvement: NOT CLAIMED; the final p95/p99/max regress against
  P1-02 and are reported explicitly.

