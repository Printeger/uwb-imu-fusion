# P1-05 gates not run

## ThreadSanitizer

`NOT_RUN`: the installed GCC 9.4 toolchain cannot link a trivial
`-fsanitize=thread` probe.  The linker reports:

```text
/usr/bin/ld: cannot find libtsan_preinit.o
```

No TSan-clean claim is made.  Deterministic fixed-slot, exception-barrier,
nested-pool rejection and one/two/four-worker behavior are covered by ordinary
focused and production equivalence tests, but those tests are not represented
as a substitute for TSan.
