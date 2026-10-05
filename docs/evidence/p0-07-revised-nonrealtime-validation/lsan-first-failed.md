# First LSan invocation: FAIL

Command: `ASAN_OPTIONS=detect_leaks=1:halt_on_error=1:abort_on_error=1 timeout 60s ... --gtest_filter=TrapezoidCorrelationRequiresQualification:GoldenPublicConfigAbiLayoutIsPreserved`

Result: exit 124. The process entered the previously observed stackless
`AddressSanitizer:DEADLYSIGNAL` loop and emitted about 808 MB of repeated text.
The oversized repetitive log was removed as a temporary artifact; this compact
failure record is retained and is not counted as a pass.

A second bounded invocation with a 15-second timeout and ASan `log_path`
repeated the same stackless loop and exited 124. Its roughly 198 MB repetitive
temporary log was also removed. LSan is therefore **FAIL**, not retried as a
pass; ASan+UBSan remains independently PASS on the complete eight-test binary.
