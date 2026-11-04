# Pixi environment

The project uses a lockfile-backed Pixi environment named `habitat`. It pins
Python 3.9 for Habitat-Sim 0.3.3 and installs the reconstruction, capture,
RDF, test, and dashboard dependencies.

## Usage

```bash
pixi install -e habitat
pixi run -e habitat smoke
pixi run -e habitat test
# Full simulator integration tests require the Replica scene:
pixi run -e habitat test-simulator
pixi shell -e habitat
```

Panel 1.4.5 and Bokeh 3.4.3 provide the experiment dashboard and self-contained
HTML plot exports. Both are pinned for the workspace's Python 3.9 runtime.

## Environment details

- `pixi.toml` uses the `conda-forge` and `aihabitat` channels and pins
  `habitat-sim` 0.3.3 with Bullet support.
- NumPy is pinned to 1.26.4 to match Habitat-Lab.
- Habitat-Lab and the local `simulator` package are installed in editable mode.
- Linux and macOS environments include their respective OpenMP runtime
  dependencies for Open3D.

## Software rendering on Linux / WSL

If Habitat cannot create its CUDA/EGL graphics context, Mesa software
rendering can be used without changing system drivers:

```bash
pixi run -e habitat collect-software
```

This task runs `load.py` with `LIBGL_ALWAYS_SOFTWARE=1`,
`EGL_PLATFORM=surfaceless`, and `HABITAT_GPU_DEVICE_ID=-1`. It still needs
the Replica scene (`pixi run -e habitat fetch-replica`) and an accessible
desktop display for the preview. Rendering uses the CPU and may be slower.
Capture settings and output paths follow `configs/second_floor.yaml`.
