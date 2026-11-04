# HW1 rubric audit — student 112006220

This records tested evidence and limitations. It does not guarantee a grade.

| Requirement | Evidence | Result |
|---|---|---|
| Manual depth unprojection from capture intrinsics | NumPy pinhole equations in `utils.py`; camera dimensions/FOV from each capture | Implemented and geometry-tested; no Open3D projection call |
| Depth units and colors | Integer scale from `depth_scale`, or the provided collector's millimetre convention when missing; floating depth remains metres; BGR converted to RGB | Tested; supplied intrinsics omits scale |
| Voxelization, normals, FPFH | Registration voxel 0.05 m; normal/FPFH radii 0.10/0.25 m; 30/100 neighbors | Implemented and tested |
| Global RANSAC | Seed 0; 3 samples; edge/distance checks; 100000 maximum trials; confidence 0.999 | Exercised in all four recorded runs |
| Open3D point-to-plane ICP | 0.075 m gate; 100 maximum iterations; default relative tolerances 1e-6 | Known-transform test passes; real data failures disclosed |
| Frame-to-frame pose composition and map integration | Current-to-previous transforms chained into first camera frame; map-only voxel size 0.025 m | Composition and map-independence tests pass |
| GT only for evaluation | No GT read in reconstruction or selection queries; scorer changes reference frame without fitting predictions | Axis and selected-frame association tested; GT also validates capture/route metadata |
| Mean L2 | Mean Euclidean camera-center error in metres | Saved trajectories rescored; metrics match recorded run values |
| Red estimate / black GT; ceiling removed | Shared display geometry and map-only 1 m ceiling band removal | Figures rendered and inspected for both phases |
| Valid experiment Turtle and sealed declarations | `api.read_experiment` verifies syntax, digest, and complete measurements | Phase 1: 1522/1522; Phase 2: 3098/3098, no pending factors |
| Baseline and selected run records | Two runs per phase; saved metrics agree with Turtle | All four runs validated |
| Evidence-based intervention | Phase 1 fitness-guided decimation; Phase 2 RGB-defined upstairs scope plus voxelized-point support | Both tested; unsuccessful matched-frame accuracy outcomes reported honestly |
| Fair comparison under frame selection | `scripts/compare_common_frames.py`; both runs preserve first-camera anchor | Phase 1: 0.058156 vs 0.103037 m; Phase 2: 0.254445 vs 0.365944 m on identical retained frames |
| Reproducibility | Pixi lock, seeds, settings/trajectory/map exports; collector replay and CLI integration | 40 unit/import/capture-control tests plus 8 simulator integration tests (48 total) pass; short seeded repeat identical; full fresh-machine repetition not claimed |
| Student Phase 2 collection | Main route03: 388 matched images/poses; 474.091 s collection; 28.712 m sampled travel | Validated; earlier stationary pilots preserved and excluded as floor captures |
| Scene coverage and quality | Images and maps show bedroom/corridor/landing/stair geometry; source includes stair descent | Quality limitation: drift and possible incomplete coverage; no numeric completeness or perfect-grade claim |
| Navigation limitation | Provided navmesh has 2 disconnected walkable islands; no path from staircase endpoint to lower-floor region | Verified without altering the supplied navmesh |
| English concise PDF report | `report.pdf` and editable `report.md` | Seven pages with implementation, settings, actual results, figures, sources, three questions, and limitations |
| Tables and plots | Four runs with Mean L2, frame counts, elapsed time, settings; eight-factor figures for each phase and HTML dashboards | Included; selection scope and timing conventions explicitly disclosed |
| Optional custom ICP bonus | `my_local_icp_algorithm` remains unimplemented | Not attempted; no bonus claimed |

## Important interpretation

Phase 2's aggregate Mean L2 falls from 2.092883 m to 0.365944 m after selecting
262 upstairs observations. Those scores cover different route scopes. On the
same 262 frames, the original baseline is better (0.254445 m). The report does
not misrepresent the aggregate reduction as repaired registration. Likewise,
Phase 1 selection worsens matched-frame accuracy. Both unsuccessful
interventions remain useful, reproducible experimental findings.

The existing `total_execution_seconds` metric measures CLI argument processing
through reconstruction and image generation. It excludes interpreter imports,
subsequent artifact serialization, and Turtle append. The report states that
scope instead of claiming whole-process wall-clock timing or repeated averages.
Reconstruction-only times are also retained in the JSON sidecars.

## Before submitting

- Read the English report personally, confirm the route descriptions, and
  understand the conclusions. The current results do not establish perfect
  floor coverage or maximum reconstruction-quality marks.
- Include source files needed by the workflow, ontology, README, and final PDF
  using the handout ZIP structure. Local imports include `plot.py` and the
  `packages/simulator` package; retain setup/configuration instructions.
- Exclude Replica assets, captures, experiment Turtle files, and generated
  diagnostics from the submission as directed by the handout.
- Use `112006220_hw1_report.pdf`; the ZIP stores it as `report.pdf`. Optional bonus code
  is not part of the completed base implementation.

## Subsequent full-frame improvement

See `docs/improving-reconstruction.md` and the report appendix. A third
experiment, `phase2_temporal_112006220`, records 3098 complete factors and
an improved full 388-frame baseline (0.390671 m). Earlier seals and runs remain
valid. The original subset interventions above are historical results; the new
full-frame baseline is a different, explicitly recorded registration treatment.
All 48 tests pass. Corrected above-floor rendering and an estimated-pose TSDF
surface improve map readability independently of trajectory scoring.
