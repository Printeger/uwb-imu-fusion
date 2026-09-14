# own-vicon initialization fix evidence index

Run root: `experiments/results/own-vicon-flow-initfix-20260912A`

| command/check | exit | result |
|---|---:|---|
| build `test_trilaterate` | 0 | target built |
| build requested nonexistent `run_ie_paper` target | 2 | retained command-name error; no scientific task |
| build `uwb_imu_fgo_paper_runner` | 0 | runner rebuilt |
| direct C++ targeted/regression tests | 0 | 32 passed |
| Python RR + own-vicon tests | 0 | 24 passed |
| `run_own_vicon_flow.py prepare` | 0 | cache and lock published |
| `run_own_vicon_flow.py preflight` | 0 | PASS |
| `run_own_vicon_flow.py execute` | 0 | complete with retained method failures |
| producer | 1 | Stage2 fixed-checkpoint recovery budget exhausted |
| robust_cauchy | 0 | valid estimate exported |
| SFUISE-ToA | 0 | valid estimate exported |
| standard evaluate | 1 | retained heterogeneous CSV field serialization failure |
| post-seal evaluator wrapper | 0 | metrics written from unchanged scientific artifacts |
| `run_own_vicon_flow.py verify` | 0 | `LOCK_CORE_INPUT_IMPLEMENTATION_PASS` |

Key SHA-256 values:

```text
e7ef707689313105d5c20154008a5b79e38b1081f0d0ec1080dc28861053c6e1  src/initializer.cpp
e912c2a3894dac338a49db2b233b6e07b46b389c95f45e06d2579c23196f5713  include/uifgo/initializer.h
4deb39830070bbb0e215c558f012238305114d5975666434c39dda3c708ae51e  test/test_trilaterate.cpp
7433414fdb60e6f667e890920e6e5cfda88324393c3d3d47eb349e3047a72cba  experiments/OWN_VICON_INITIALIZATION_FIX_PROTOCOL.md
2920d9b97d123349c9cf0de21c26c21556cb0c5eb3f0e4f500b81bfd8e5fb804  lock.json
456acdbe2f1bf8b4dad62dce155027a7216276915747d5a772f05ddd8245ce39  metrics.csv
7532d1cbd1038ce942cdd33223a4138c1184a9303eb3b87733be98a73113f0aa  paired_differences.csv
```

The run lock additionally contains the exact command arrays, input/config/binary allowlists, source bag SHA-256,
implementation hashes, method artifacts and wall times. The evaluator-private GT/range reference remains outside the
estimator namespace.

Commands executed from the repository/workspace were:

```text
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_fgo --target test_trilaterate run_ie_paper -j2
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_fgo --target uwb_imu_fgo_paper_runner -j2
/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/test_trilaterate --gtest_color=no
/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/test_config --gtest_color=no
/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/test_optimizer --gtest_color=no
/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/test_graph_builder --gtest_color=no
/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/test_paper_input --gtest_color=no
(cd experiments/scripts && python3 -m unittest test_recover_vs_reject.py test_own_vicon_flow.py)
python3 experiments/scripts/run_own_vicon_flow.py prepare --run experiments/results/own-vicon-flow-initfix-20260912A
python3 experiments/scripts/run_own_vicon_flow.py preflight --run experiments/results/own-vicon-flow-initfix-20260912A
python3 experiments/scripts/run_own_vicon_flow.py execute --run experiments/results/own-vicon-flow-initfix-20260912A
python3 experiments/scripts/run_own_vicon_flow.py evaluate --run experiments/results/own-vicon-flow-initfix-20260912A
python3 experiments/scripts/evaluate_own_vicon_flow_postsealed.py --run experiments/results/own-vicon-flow-initfix-20260912A
python3 experiments/scripts/run_own_vicon_flow.py verify --run experiments/results/own-vicon-flow-initfix-20260912A
```
