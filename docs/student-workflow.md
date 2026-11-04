# HW1 workflow for student 112006220

The two phases use the same implementation in `utils.py`. Ground truth is
used only by trajectory evaluation. Default registration uses FPFH/RANSAC
followed by Open3D point-to-plane ICP. Depth unprojection uses NumPy and the
capture's own `intrinsics.json`; integer depth defaults to millimetres and
an explicit `depth_scale` overrides that default. Floating depth is already
in metres.

## Phase 1

The provided archive was downloaded from Google Drive ID
`17dnyifA0LOi6CXWGOYgl3db1WSuOOHbl` into `eval/_downloads/phase1.zip`.
It is extracted unchanged under `eval/first_floor/`. The actual archive has
**191 RGB/depth pairs and 191 GT poses**, rather than the 152-frame example
shown in the handout. Do not discard data just to match that example.

The first experiment is `experiments/phase1_112006220/`. Its declaration
records hypotheses before the baseline is measured. For a fresh experiment:

```bash
pixi run -e habitat python api.py declare --name phase1_new --data-dir eval/first_floor --floor 1
# Edit the declaration label and prediction comments BEFORE measuring.
LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless pixi run -e habitat python reconstruct.py --experiment experiments/phase1_new --no-vis
pixi run -e habitat python api.py explore experiments/phase1_new
```

Inspect all eight factor timelines/distributions and the baseline image.
Write a SPARQL query in that experiment's `queries/` directory based on the
measured evidence. Check the returned frame list before reconstruction:

```bash
pixi run -e habitat python api.py query experiments/phase1_new --query-file experiments/phase1_new/queries/frames.rq
LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless pixi run -e habitat python reconstruct.py --experiment experiments/phase1_new --selection-query experiments/phase1_new/queries/frames.rq --no-vis --write
```

`--write` is required to record a selected run in Turtle. Compare error,
retained frame fraction, map coverage, and runtime. Do not edit the sealed
measurements or claim that low registration RMSE alone proves correctness.
Removing frames can create large motion gaps and worsen reconstruction.
A changed reconstruction treatment should use a new experiment name.

## Phase 2: collect your own observations

The Replica apartment asset is installed separately from the Phase 1 capture.
Start the student collector using the verified software renderer:

```bash
pixi run -e habitat collect-software --config configs/second_floor.yaml --output-root eval/_data/second_floor/112006220
```

Use W/S to move, A/D to turn, C/Space to capture, Q/Escape to finish.
Capture small, overlapping viewpoint changes, including intermediate views
while turning. Cover the major rooms, connecting spaces, and corners;
finish with Q/Escape so `GT_pose.npy` and `capture_metadata.json` are saved.
The metadata records the actual configuration, frame count, collection duration,
sampled path length, and route bounds; GT poses preserve the sampled route.
Add a human-readable description of rooms visited in
`docs/phase2-capture-notes.md`. This is manual student data collection;
the bundled short rotation test trajectory does not meet floor-coverage
requirements.

The configuration has `output.clear_existing: true`: use a NEW output path
for another session if you want to retain the previous capture. Do not start
another collector on an existing capture path without preserving it first.

After completing the capture:

```bash
pixi run -e habitat python api.py explore eval/_data/second_floor/112006220_route03
pixi run -e habitat python api.py declare --name phase2_112006220 --data-dir eval/_data/second_floor/112006220_route03 --floor 2
# Fill the declaration label and prediction comments before the next command.
LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless pixi run -e habitat python reconstruct.py --experiment experiments/phase2_112006220 --no-vis
pixi run -e habitat python api.py explore experiments/phase2_112006220
```

Then inspect factors and test interventions using the same procedure as Phase 1.
Do not use GT to initialize registration, tune individual transforms, or fit
predicted trajectories to the reference.

## Outputs and report

Each reconstruction saves `map_baseline.ply` / `map_selected.ply`,
`trajectory_baseline.npy` / `trajectory_selected.npy`, and
`metrics_baseline.json` / `metrics_selected.json` in its experiment directory.
The JSON includes frame count and consumed frame identifiers, Mean L2 in metres, measured reconstruction and
execution time, seed, voxel sizes, registration settings, and query path.
`null` Mean L2 means no valid score was available. Execution time is measured
through reconstruction and image generation, before artifact serialization;
use `--no-vis` for comparisons so manual window interaction is excluded.
The map-only voxel size defaults to 2.5 cm and does not affect pose estimation.

To export a CSV comparison table and a static factor plot from either phase:

```bash
pixi run -e habitat python scripts/summarize_experiment.py experiments/phase1_112006220
```

Use the baseline/selected reconstruction screenshots and the four HTML plots
from `api.py explore`. Explain both successful and unsuccessful interventions.
Compare both floors with an English report table containing Mean L2, frame
count, execution time, and tested hyperparameters. Broad floor coverage must
be checked visually, not inferred from Mean L2 alone.

Keep captures, Replica assets, experiment Turtle, and generated plots out of
the submission ZIP as directed by the handout. Include all source modules used
by the submitted workflow (including `plot.py` and `packages/simulator` if needed),
`ontology/hw1.ttl`, README, and your final English `112006220_hw1_report.pdf` (named `report.pdf` inside the ZIP).
The optional custom ICP remains a separate bonus and is not implemented by
this required-pipeline change.

## Explore existing results without rerunning registration

```bash
pixi run -e habitat python scripts/view_reconstruction.py experiments/phase1_112006220 --mode baseline
```

The Open3D window lets you rotate, pan, and zoom the saved map. Close the
window to return to the terminal. Add `--mode selected` for the intervention.
The interactive apartment simulator and this reconstructed point-cloud viewer
serve different purposes: collect observations in the simulator; inspect the
estimated map in the viewer.

## Easier student collection

Optional auto-capture saves after movement/turn actions that change the camera
pose, including held movement keys. It avoids requiring C after every step:

```bash
pixi run -e habitat collect-software --auto-capture --output-root eval/_data/second_floor/112006220_route03
```

Use a fresh path for each session. Do not rerun this command on an existing
capture because the configuration clears the selected output directory.
Stationary repeats do not establish floor coverage. Inspect the changing
capture counter while navigating and press Q/Escape to finish and save GT.

## Report generation and audit

`docs/rubric-audit.md` tracks verified requirements and remaining work.
Generate the current English PDF/Markdown draft with:

```bash
pixi run -e habitat python scripts/build_report.py
```

The script checks recorded metrics against sealed Turtle results. A final
build (`--final`) requires measured baseline and selected runs for Phase 2;
student route/coverage review and a completed cross-phase discussion are
still necessary. The present draft clearly marks missing Phase 2 evidence.

For a fair error comparison on the same retained frames, run:

```bash
pixi run -e habitat python scripts/compare_common_frames.py experiments/phase1_112006220
pixi run -e habitat python scripts/compare_common_frames.py experiments/phase2_112006220
```

These commands evaluate saved trajectories without changing registration or
sealed factor evidence. They currently require the same first-camera anchor
in both runs. They prevent a changed route scope from being mistaken for a
registration improvement. The report's final build requires these comparisons.

The main student capture is now `eval/_data/second_floor/112006220_route03`
(388 frames). Its upstairs and stairwell scope is recorded in the capture
notes. The provided navmesh has disconnected upper/lower walkable regions;
the staircase endpoint has no path to the lower living-room region.

Both phases now have measured baseline/selected runs and common-frame controls.
The seven-page English `report.pdf` is built from these records; `report.md`
is the editable source. Read `docs/rubric-audit.md` for quality and scope
limitations. Existing experiment names are append-only; use a new name for
a new treatment.
