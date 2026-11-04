# Reconstruction improvement results

The original experiments remain intact. The improved Phase 2 treatment is
`experiments/phase2_temporal_112006220/`.

| Full capture | Original Mean L2 | Recommended result | Frames |
|---|---:|---:|---:|
| Phase 1 | 0.05883481 m | 0.05883481 m (keep standard) | 191 |
| Phase 2 | 2.09288325 m | 0.39067118 m (temporal profile) | 388 |

Phase 2 improves **81.33% on identical full-frame input**, without cropping
away the stairs. On the same 262 upstairs frames used in the older selection
analysis, standard poses score 0.25444502 m and temporal poses score
0.20926940 m. These are evaluation-only comparisons of saved estimates.

Phase 1 trials with finer ICP (0.06048285 m), temporal ICP (0.24179532 m),
and 1 cm Huber loss (0.06368346 m) were worse than its standard baseline.
They are not adopted. Pilot sources and measured outcomes are preserved in
`experiments/registration_pilots/`; those diagnostics are not mislabeled as
sealed homework baseline runs.

## Corrected map viewing

The old image-axis-to-Y-up transform was correct, but the display camera's
`front=[0,-1,0]` placed it below the floor. That obscured furniture and room
geometry. Views now use `front=[0,1,0]`, centered bounds, and a landscape
orientation. This rendering correction changes no poses or scores. Earlier
report views are preserved in `report_assets/previous_views/`.

## Temporal profile

All frames still use manual NumPy unprojection, voxel downsampling, normals,
FPFH/RANSAC, and Open3D point-to-plane ICP. The optional temporal profile runs
ICP from three initializations: RANSAC, identity, and the previous **estimated**
relative motion. It rejects non-finite and zero-support proposals and relative
motions exceeding 1 m or 60 degrees. Among proposals within 0.01 of the best
fitness it minimizes:

```
inlier_rmse + 0.005 * norm(translation_change_from_prior)
            + 0.005 * rotation_change_from_prior_in_radians
```

This is a sequential-overlap assumption, not a general large-jump matcher.
The standard profile remains available and is better on the Phase 1 capture.
If every ICP proposal fails, an explicit evaluated prior-motion fallback is
recorded in diagnostics. No such fallback occurred in the improved Phase 2
run: 141 RANSAC, 114 identity, and 132 previous-motion starts were selected.
GT does not enter proposal generation, selection, or motion bounds.

The recorded result reproduces the pilot camera matrices exactly. Its 3098
factor occurrences and seal validate. All preprocessing and coarse RANSAC
factor values exactly match the original baseline; only local registration
acceptance changes. Additional ICP starts cost computation. Single elapsed
measurements were 278.694 s (original) and 471.559 s (temporal), with concurrent
work during the latter, so these are not isolated speed benchmarks.

`registrationProfile` is recorded as a reconstruction setting in the ontology.
Old experiments without that setting retain `standard` behavior. A CLI profile
that disagrees with a declaration or sealed experiment is rejected. Previews
now save to `*_preview` artifacts, preserving recorded results.

## Surface fusion and viewing

The improved experiment also contains `surface_baseline.ply`: a colored TSDF
surface using the saved estimated camera matrices, 2 cm voxels, 8 cm SDF
truncation, and 10 m depth truncation. A display crop removes geometry above
the initial camera height. Raw merged clouds remain available. Fusion changes
map representation, never trajectory estimates or Mean L2; its settings and
estimated-pose hash are saved in `surface_baseline.json`.

Open the improved map:

```bash
pixi run -e habitat view-map experiments/phase2_temporal_112006220 --surface
```

Open the original Phase 1 map with the corrected above-floor view:

```bash
pixi run -e habitat view-map experiments/phase1_112006220 --mode baseline
```

To reproduce surface fusion:

```bash
pixi run -e habitat python scripts/fuse_map.py \
  --capture eval/_data/second_floor/112006220_route03 \
  --camera-poses experiments/phase2_temporal_112006220/camera_poses_baseline.npy \
  --metrics experiments/phase2_temporal_112006220/metrics_baseline.json \
  --output experiments/phase2_temporal_112006220/surface_baseline.ply
```

For a new temporal experiment, declare a new name, fill the prediction/label,
and add this setting above the machine marker **before** reconstruction:

```turtle
exp:YOUR_NEW_NAME hw1:hasFactorSetting [
  a hw1:FactorSetting ;
  hw1:settingParameter hw1:registrationProfile ;
  hw1:settingRole hw1:ReconstructionSetting ;
  hw1:settingForFactor hw1:ReconstructionRun ;
  hw1:settingValue "temporal"^^xsd:string
] .
```

Then run `reconstruct.py --experiment experiments/YOUR_NEW_NAME --no-vis`.
Do not edit or overwrite the old sealed experiment to change its treatment.

## Verification and limits

The final full suite passes **48/48 tests**, including real-scene simulator
integration. The new tests cover rejection of implausible high-fitness updates,
explicit zero-support fallback, state isolation, and estimated-pose export.
A separate small CLI integration check confirms that previews preserve the
recorded Turtle, map, trajectory, metrics, and image (rendering stubbed in that
file-routing test; actual map images were separately rendered and inspected).

These are genuine improvements, not a promise of zero error or perfect
coverage. No GT-based alignment fit or GT-initialized registration is used.
Depth noise and poorly constrained views still limit accuracy. The updated
report separates original experiments, unsuccessful trials, improved full-frame
accuracy, and map-presentation changes.
