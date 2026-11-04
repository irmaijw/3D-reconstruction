# Phase 2 capture notes — 112006220

## Preserved pilot attempts

- `eval/_data/second_floor/112006220`: 8 saved frames, one unique pose.
- `eval/_data/second_floor/112006220_route02`: 2 saved frames, one unique pose.
- Both have valid RGB/depth/intrinsics/GT files but no spatial coverage across
  the saved views. The second attempt involved movement before saving both
  frames at its final viewpoint. Neither is presented as a floor reconstruction.

## Main student capture

- Path: `eval/_data/second_floor/112006220_route03`
- Scene: Replica `apartment_0`; configuration `configs/second_floor.yaml`
- Renderer: Mesa software (`collect-software`)
- Mode: manual student navigation with optional automatic capture after changed
  movement/turn poses; input remains student-controlled.
- Saved observations: **388** matched 512x512 RGB/depth pairs and **388** GT poses.
- Collection duration: **474.091 s**; sampled camera path length **28.712 m**.
- Sampled XZ bounds: X **[-0.202, 5.398] m**, Z **[-0.975, 6.546] m**.
- Distinct rounded poses: **333**. Revisited poses are retained in the capture;
  automatic capture suppresses unchanged blocked actions, not later revisits.
- Route scope observed in RGB images: starting bedroom, adjoining bedroom,
  upstairs corridor/landing, and stairwell descent toward the lower entrance.
  The student reported difficulty reaching the living room downstairs. A direct
  navigation-mesh check found two disconnected islands: the staircase end is
  on island 1, with no path to lower-floor island 0. Their areas are about
  33.419 and 55.577 square metres. This is a supplied navigation-mesh limitation,
  not evidence of missing keyboard input. That
  room is not claimed as covered. Full-room coverage requires map inspection.
- Camera heights span **0.725–2.925 m**; the route is not exclusively horizontal
  upstairs motion. Its stairwell extension is retained and disclosed, rather
  than silently filtered using GT.
- Uncertainty-zone counts: benign **57**, moderate **100**, severe **0**,
  outside **231**. Absence from the severe zone limits comparisons of its effect.
- Camera movement between consecutive saved positions: maximum **0.600 m**.
  Multiple key events may be consumed before one render/save; this is a real
  capture condition to consider during registration diagnostics.
- Baseline experiment: `experiments/phase2_112006220`
- Parameter settings, route poses and duration: `capture_metadata.json`,
  `GT_pose.npy`, and experiment metrics sidecars.
- GT is reserved for scoring and capture/route validation; it is not used to
  initialize registration, fit predicted poses, or choose reconstruction frames.

## Analysis status

Baseline and selected reconstructions are complete. All 3098 baseline factor
occurrences and both run records validate. The selected query retains 262
upstairs frames through frame 270 with voxelized yield >= 2000. Its lower
aggregate error is due to changed route scope; matched-frame error worsens
from 0.254445 m to 0.365944 m. See the experiment analysis and final report.
