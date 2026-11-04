# PhysicalAI26 HW1

This repository contains the HW1 capture simulator and a reconstruction and
experiment workflow. Python dependencies are managed by [Pixi](https://pixi.sh/)
using the checked-in `pixi.lock` file.

## Setup

1. Install Pixi using the instructions at [pixi.sh](https://pixi.sh/).
2. Restore the pinned Habitat-Lab dependency if you extracted the submission ZIP
   (the upstream dependency is not bundled):

   ```bash
   git clone https://github.com/facebookresearch/habitat-lab.git dependencies/habitat-lab
   git -C dependencies/habitat-lab checkout 0fb6f43ffe806a8088a171b036336c093bcf604e
   ```

   From this directory, create the `habitat` environment:

   ```bash
   pixi install -e habitat
   ```

   The environment provides Python 3.9, Habitat-Sim 0.3.3 with Bullet,
   Habitat-Lab, and the project dependencies. The first install can take a few
   minutes.
3. Check that Habitat-Sim and Habitat-Lab import correctly:

   ```bash
   pixi run -e habitat smoke
   ```

4. Download and extract the Replica apartment scene used by the default
   configuration:

   ```bash
   pixi run -e habitat fetch-replica
   ```

   This creates `replica_v1/apartment_0/`. Keep the scene at that path for the
   default `configs/second_floor.yaml` configuration.

## Capture data

Start the interactive Habitat-Sim collector:

```bash
pixi run -e habitat python load.py
```

The default configuration is `configs/second_floor.yaml`. Captures are written
under `eval/_data/second_floor/` and include RGB and depth PNGs, camera
intrinsics, and a ground-truth pose trajectory. Use `--config` to select a
different configuration or `--output-root` to change the output directory. See
[`docs/load.md`](docs/load.md) for controls, replay mode, and capture details.

## Reconstruction and experiments

Run commands from this repository directory with `pixi run -e habitat`. The
experiment workflow starts by declaring an experiment over a capture, then
reconstructing it:

```bash
pixi run -e habitat python api.py declare \
  --name <experiment-name> \
  --data-dir <capture-directory> \
  --floor 2

pixi run -e habitat python reconstruct.py \
  --experiment experiments/<experiment-name> \
  --no-vis

pixi run -e habitat python api.py explore experiments/<experiment-name>
```

`declare` creates an experiment directory, the first reconstruction measures
pipeline factors and records a baseline, and `explore` displays the results and
writes interactive plots. For the full workflow, SPARQL frame selection, and
query examples, see [`docs/e2e-workflow.md`](docs/e2e-workflow.md).

## More documentation

- [`docs/pixi.md`](docs/pixi.md): environment details and Pixi commands
- [`docs/load.md`](docs/load.md): capture settings, controls, and outputs
- [`docs/reconstruct.md`](docs/reconstruct.md): reconstruction options
- [`docs/api.md`](docs/api.md): experiment and inspection commands
- [`docs/e2e-workflow.md`](docs/e2e-workflow.md): end-to-end walkthrough

## Student workflow

For the downloaded Phase 1 capture, evidence-based intervention procedure,
Phase 2 collection path for student 112006220, and report artifacts, see
[`docs/student-workflow.md`](docs/student-workflow.md).

On this WSL setup, use `pixi run -e habitat collect-software` for the
simulator preview. This uses Mesa CPU rendering.

To explore a saved reconstruction without recomputing registration:

```bash
pixi run -e habitat view-map experiments/phase1_112006220 --mode baseline
```

Rotate/pan/zoom in the Open3D window and close it when finished. The second-floor
simulator also supports `collect-software --auto-capture` to save changing views
while you navigate. Use a fresh `--output-root` for each capture.

The final English report is `112006220_hw1_report.pdf`, with editable source
`112006220_hw1_report.md`. It uses Times New Roman 12 pt, justified paragraphs,
and 1.5 line spacing. Historical report files have been removed.

Build the submission package with:

```bash
python3 scripts/package_submission.py
```

The resulting `112006220_hw1.zip` contains the final PDF under the handout's
required archive name `report.pdf`, with source files at the ZIP root.
It excludes captures, Replica assets, experiment Turtle files, diagnostics,
environments, and caches. Original measurements remain local in `experiments/`.
The custom ICP bonus was not attempted. The required backend is Open3D.

## Improved full-frame reconstruction

See [`docs/improving-reconstruction.md`](docs/improving-reconstruction.md) for
the recorded temporal ICP experiment, corrected above-floor views, and TSDF
surface fusion. Phase 2 uses all 388 frames and improves Mean L2 from
2.092883 m to 0.390671 m; Phase 1 retains its better standard baseline.

```bash
pixi run -e habitat view-map experiments/phase2_temporal_112006220 --surface
```
