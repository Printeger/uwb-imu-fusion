# P1-02 explicit NOT_RUN list

No P1-02 task-required test or performance item is `NOT_RUN`.

Hardware calibration/qualification and P0-07 independent validation were not
reopened.  They remain outside R08 and are not claimed as PASS; P0-07 remains
`VALIDATION_PAUSED`, `formal_eligible=false`, and no `golden-p0-07` tag exists.

TSan and a new sanitizer campaign were not required for P1-02 and were not
run.  The complete 34-test CTest suite, including the retained P0-07 evidence
and atomic-failure runners, was run and passed.
