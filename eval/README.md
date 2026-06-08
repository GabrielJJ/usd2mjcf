# MuJoCo drop evaluation

This directory contains small scripts that evaluate MJCF models in MuJoCo and record short videos of the simulation.
For a workflow-oriented guide, see [docs/07-evaluating-mjcf.md](../docs/07-evaluating-mjcf.md).

## Scripts

| Script | Description |
|--------|-------------|
| `drop_test.py` | Drop a single object from a given height. |
| `stack_spawn_test.py` | Spawn multiple copies sequentially, stacked one above the other with small rotations. |

## Prerequisites

1. Install the evaluation dependencies:

```bash
pip install -r eval/requirements.txt
```

2. Provide a complete MJCF/XML file with at least one geom under a top-level body in `worldbody`. The converter typically names this body `RootNode`; minimal test models may use `Root`. For assets converted from USD in this repository, generate collision geometry first:

```bash
python test/usd2mjcf_test.py assets/bin_b04.usda --generate_collision
```

The converter writes MJCF files under `assets/MJCF/`.

## Drop test (`drop_test.py`)

Run the drop test from the repository root:

```bash
python eval/drop_test.py --xml path/to/model.xml
```

You can also pass the XML path as a positional argument:

```bash
python eval/drop_test.py path/to/model.xml
```

### Examples

Default sample asset (`assets/MJCF/bin_b04.xml`):

```bash
python eval/drop_test.py
```

Custom XML with explicit output path:

```bash
python eval/drop_test.py --xml assets/bin_b04/MJCF/bin_b04.xml --output eval/output/bin_b04_drop.gif
```

Drop from a higher starting point with a longer simulation:

```bash
python eval/drop_test.py --xml assets/bin_b04/MJCF/bin_b04.xml --height 2.0 --duration 5.0
```

Open the MuJoCo viewer while simulating:

```bash
python eval/drop_test.py --xml assets/MJCF/bin_b04.xml --viewer
```

## Options

| Flag | Default | Description |
|------|---------|-------------|
| `xml` (positional) | — | MJCF/XML file to test. Alternative to `--xml`. |
| `--xml` | `assets/MJCF/bin_b04.xml` | MJCF/XML file to test. |
| `--root-body` | auto | Top-level body to drop (`Root`, `RootNode`, or the only worldbody child). |
| `--height` | `1.0` | Spawn height in meters above the ground plane. |
| `--duration` | `3.0` | Simulation length in seconds. |
| `--fps` | `30` | Frame rate of the output video. |
| `--output` | `eval/output/<xml_stem>_drop.mp4` | Output video path (`.mp4` or `.gif`). |
| `--viewer` | off | Open the MuJoCo passive viewer during simulation. |

If both a positional XML path and `--xml` are given, they must match.

## Stack spawn test (`stack_spawn_test.py`)

Spawns several copies of the same MJCF model in sequence. Each copy appears above the previous one after a delay, with incremental yaw and pitch rotations. Spawn heights are precomputed from the object bounds so copies form a vertical tower (`base-height`, then `base-height + object_height + gap`, and so on).

```bash
python eval/stack_spawn_test.py --xml path/to/model.xml
```

### Examples

Three boxes spawning every second:

```bash
python eval/stack_spawn_test.py --xml eval/output/smoke_complete.xml --count 3 --spawn-interval 1.0
```

Four bins with stronger rotation and longer simulation:

```bash
python eval/stack_spawn_test.py \
  --xml assets/bin_b04/MJCF/bin_b04.xml \
  --count 4 \
  --spawn-interval 1.5 \
  --rotation-step 15 \
  --tilt-step 5 \
  --duration 12 \
  --output eval/output/bin_b04_stack.gif
```

### Options

| Flag | Default | Description |
|------|---------|-------------|
| `xml` (positional) | — | MJCF/XML file to replicate. Alternative to `--xml`. |
| `--xml` | `assets/MJCF/bin_b04.xml` | MJCF/XML file to replicate. |
| `--count` | `3` | Number of copies to spawn. |
| `--spawn-interval` | `1.0` | Seconds between each spawn. |
| `--base-height` | `1.0` | Bottom height for the first object. |
| `--stack-gap` | `0.05` | Vertical gap added above the current stack. |
| `--rotation-step` | `12.0` | Yaw increment in degrees per spawn index. |
| `--tilt-step` | `4.0` | Pitch increment in degrees per spawn index. |
| `--root-body` | auto | Top-level body name in the source XML. |
| `--duration` | `8.0` | Simulation length in seconds. |
| `--fps` | `30` | Frame rate of the output video. |
| `--output` | `eval/output/<xml_stem>_stack_spawn.mp4` | Output video path (`.mp4` or `.gif`). |
| `--viewer` | off | Open the MuJoCo passive viewer during simulation. |

## What the drop test does

1. Loads the MJCF/XML model.
2. Detects the object root body (`Root`, `RootNode`, or the sole top-level body).
3. Ensures that body has a free joint so the object can fall.
4. Adds a ground plane for contact.
5. Aligns the lowest point of the object to the requested spawn height.
6. Simulates gravity, records frames, and writes a video.
7. Prints summary metrics (contact time, final height, max downward velocity).

## Headless rendering

When no display is available, set an offscreen MuJoCo backend before running:

```bash
export MUJOCO_GL=egl   # or osmesa
python eval/drop_test.py --xml path/to/model.xml
```

## Output

Videos are written to `eval/output/` by default. The directory is created automatically if it does not exist.
