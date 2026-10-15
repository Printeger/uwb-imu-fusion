# SC8 advisor presentation: figure sources

Main deck: exactly 15 slides. Frozen solver: `61907ab417450d317e443e4ff10f2cce062deabe`.

Charts are rearranged from the original frozen SC8 SVG/summary evidence, with new SVG assets and 300-dpi PNG fallbacks embedded in PPTX. All native architecture nodes, arrows, tables, takeaways and advisor questions remain editable.

The report and plotdata are read-only. `FIGURE_SOURCES.json` records input and output hashes; `NUMERICAL_CLAIMS.json` records exact paths, arithmetic and formatting for numerical claims. A process SUCCESS is never interpreted as full trajectory completion.

| Slide | Asset / original evidence | Section and displayed metric | Caveat |
|---|---|---|---|
| 3 | s03_healthy / F01_paired_accuracy.svg | SC8_RESULTS E01: three healthy seeds, per-run output-stamp APE | Different initialization information and output stamps; S-FINAL is same-run noncausal valid history, not batch optimum. |
| 4 | s04_vicon / F02_real_trajectories.svg | SC8_RESULTS E00/E02: R3 XY and height on registered TEST | GT-assisted TEST-involved segmented calibration; no FULL TEST output; no independent blind-test claim. |
| 5 | s05_coverage / F00_admission.svg | SC8_RESULTS E02: output coverage on full GT TEST grid | Temporal coverage only for Walk; R3 FULL NA, no real raw PL; percentages are not accuracy or commit rates. |
| 6 | s06_gains / F04_fault_paired_gains.svg | SC8_RESULTS E03: matched per-seed gains, N=3 each; black tick is mean | Negative is deterioration. Position and attitude effects separate; real physical paired gains remain unavailable. |
| 7 | s07_fault_timeline / F03_joint_fault_receipts.svg | SC8_RESULTS E03: SC8-J seed20261020 APE/ARE and native receipt states | T-SIM; source pulses fixed. Rejoined flag persists and is not proof of accuracy recovery; full SFUISE outlier scale retained. |
| 8 | s08_horizontal / F05_H_Stanford.svg | SC8_RESULTS E04: H PE vs PL actual log-count bins, full + zoom | Timely bins teal (darker=more), late bins gray. Figure N=10200 per method; overall 77131 correlated receipts is a separate aggregate. |
| 9 | s09_vertical / F05_V_Stanford.svg | SC8_RESULTS E04: V PE vs PL actual log-count bins, full + zoom | Timely bins teal (darker=more), late bins gray. Figure N=10200 per method; overall 77131 correlated receipts is a separate aggregate. |
| 9 | s09_availability / F06_AL_and_SI.svg | SC8_RESULTS E04: 3 healthy seeds, timely joint / all 600 planned epochs | Diagnostic AL only; no estimates/PL changed; no certified application requirement. |
| 10 | s10_window / F07_window_frontier.svg | SC8_RESULTS E05: 14 actual window runs, common epochs1101–2100 | LF, LI and replay coupled; same seed/stream, no universal trend; resource markers are sampled RSS, not peak. |
| 11 | s11_latency / F08_latency_and_breakdown.svg | SC8_RESULTS E06: all measured E01/E03 main core/sink latencies and exclusive means | CDF counts are frame descriptions, not independent trials; SFUISE has different capabilities/cutoffs; E06 performance repeats listed separately. |
| 12 | s12_ablation / F09_ablations.svg | SC8_RESULTS E07: raw ablation values and signed matched-FULL percent changes | G2 no qualified output. Joint+sample0/support combined reversion; M same-center witness distinct from full-trajectory A and measured P. |
| 13 | s13_long / F10_long_streams.svg | SC8_RESULTS E08: all saved long-stream APE/ARE, rolling core P95 and sampled RSS | Actual extrema/events from frozen compact data; FULL ends7171; late episode not confirmed; sampled RSS not process peak. |

Slides 1, 2, 14 and 15 use native editable diagrams / tables. Architecture follows `research/sc6_service.cpp`, `research/rolling_policy.hpp`, and the verified SC7/SC8 descriptions. Historical reports supply architecture only, never SC8 performance values.

Reproduce (Python 3.8+; Liberation Sans or installed metric-compatible font):

```bash
python -m pip install -r docs/sc8/presentation/requirements.txt
python tools/sc8_slides_figures.py
python tools/sc8_advisor_slides.py build
python tools/sc8_advisor_slides.py render --soffice /path/to/soffice
python tools/verify_sc8_advisor_slides.py
```

PDF is exported from the actual PPTX with LibreOffice Impress. The renderer also writes all 15 PNG previews and a contact sheet under ignored `.cache/sc8-slides/preview/`. It launches no experimental solver.

The referenced GOAL_SC8_ADVISOR_SLIDES.txt was absent from the supplied directory. The complete provided user objective and SC8_ADVISOR_SLIDES_REQUIREMENTS.md define the locked 15-slide storyboard; no replacement goal was fabricated.
