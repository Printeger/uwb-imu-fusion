# Archived legacy offline IE material

This directory preserves material inherited from the offline UWB–IMU
post-processing project at the branch fork point (`main@cfe6d29`). It is kept
for traceability and possible reference only.

The active realtime PL package does not reference this directory from
`CMakeLists.txt`, `package.xml`, launch files, message generation, installation,
or its test targets. Moving these files here therefore removes them from normal
catkin/ROS discovery without deleting their Git history.

Contents:

- `include/uifgo`, `src`, `tools/run_offline.cpp`: legacy batch estimator and
  offline runner
- `config`, `data`: legacy dataset/offline experiment configurations
- `launch`: offline launch files
- `test`: tests for the archived `uifgo` implementation
- `doc`: inherited offline papers, notes, TeX, and result PDF
- `msg`: message definitions unused by the realtime PL/simulator chain
- `simulator`: superseded circle-only trajectory/path helpers
- `README.mixed-pre-archive.md`: the former mixed offline/realtime root README

`msg/AuxCommand.msg` intentionally remains active at repository root because
the current simulator's `SO3Command.msg` depends on it.

## Archive validation

Validated on 2026-09-15 after the move, starting from the saved PL checkpoint
`8ada4f7`:

- package-level clean Release build: passed
- active package test results: 166 tests, 0 errors, 0 failures, 0 skipped
- 30-second no-fault figure-eight ROS closed-loop smoke: 47 processed and
  committed epochs, 0 rejected, 0 processing errors
- generated run contract: `uwb-imu-pl/v5` schema validation passed
