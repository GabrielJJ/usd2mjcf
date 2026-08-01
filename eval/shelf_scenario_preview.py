#!/usr/bin/env python3
"""Preview the toteweg-on-shelf scenario: static `corridor0` fixture (estante_l1,
estante_r1, ...) plus `toteweg` crates placed at the calibrated candidate shelf
slots (see docs/teleop_simple_study/toteweg_factory_scene_migration_plan.md,
Phase 4 "shelf spawn placement").

Builds the scene the same way MujocoSimulator._build_static_object /
_build_object do (same geom params: condim=4, friction, solref, mesh-per-convex-
piece), without depending on the full Isaac/torch SIMPLE stack -- reads the
staged asset bundles directly.

Usage:
    python eval/shelf_scenario_preview.py                  # render PNGs only
    python eval/shelf_scenario_preview.py --settle          # step physics first
    python eval/shelf_scenario_preview.py --viewer          # open interactive viewer
    python eval/shelf_scenario_preview.py --stochastic --seed 0 --settle
        # sample per-tier occupancy like src/simple/dr/shelf_group.py (0-3
        # totes/tier, >=2/shelf), non-overlapping X within each tier
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import random
from pathlib import Path

import mujoco
import mujoco.viewer

REPO_ROOT = Path(__file__).resolve().parents[3]
CORRIDOR_DIR = REPO_ROOT / "src/simple/assets/fixtures/corridor0/MJCF"
TOTEWEG_DIR = REPO_ROOT / "src/simple/assets/totes/toteweg/MJCF"

# Candidate shelf slots, calibrated from the real corridor0 shelf-board geometry
# (see the migration plan doc, Phase 4). Z already includes toteweg's
# stable-pose offset (0.0687 m) on top of the shelf board.
# NOTE: l-block (l1+l3) and r-block (r1+r2+r3) were swapped between Y-bands
# (each shelf also rotated 180 deg in place) -- l1/l3 now sit where r1/r2/r3
# used to be. facing: yaw (deg about +Z) that points the crate's front/recess
# mark toward the aisle -- 180 deg for l1/l3 (aisle is -Y from them now),
# 0 deg for r1 (aisle is +Y from it now). Still a placeholder until visually
# confirmed against the mesh's authored front axis.
SLOTS = {
    "l1-B": dict(pos=(-1.0864, 1.6861, 0.5238), yaw_deg=180.0),
    "l1-D": dict(pos=(-1.0864, 1.6861, 1.0077), yaw_deg=180.0),
    "l1-E": dict(pos=(-1.0864, 1.6861, 1.2824), yaw_deg=180.0),
    "l3-B": dict(pos=(-2.9057, 1.6861, 0.5238), yaw_deg=180.0),
    "l3-D": dict(pos=(-2.9057, 1.6861, 1.0077), yaw_deg=180.0),
    "l3-E": dict(pos=(-2.9057, 1.6861, 1.2824), yaw_deg=180.0),
    "r1-B": dict(pos=(1.1326, -0.2985, 0.6326), yaw_deg=0.0),
    "r1-C": dict(pos=(1.1326, -0.2985, 0.9593), yaw_deg=0.0),
    "r1-D": dict(pos=(1.1326, -0.2985, 1.2861), yaw_deg=0.0),
    "r1-E": dict(pos=(1.1326, -0.2985, 1.6652), yaw_deg=0.0),
}


# Same shelf geometry as SHELF_SPECS in src/simple/dr/shelf_group.py, duplicated
# here (rather than imported) so this script stays dependency-free of the full
# Isaac/torch SIMPLE stack, matching this file's existing convention of
# re-implementing MujocoSimulator's geom-building logic locally.
STOCHASTIC_SHELVES = {
    "l1": dict(x_min=-1.9961, x_max=-0.1768, y=1.6861, yaw_deg=180.0,
               tiers={"B": 0.5238, "D": 1.0077, "E": 1.2824}),
    "l3": dict(x_min=-3.8153, x_max=-1.9961, y=1.6861, yaw_deg=180.0,
               tiers={"B": 0.5238, "D": 1.0077, "E": 1.2824}),
    "r1": dict(x_min=0.1237, x_max=2.1414, y=-0.2985, yaw_deg=0.0,
               tiers={"B": 0.6326, "C": 0.9593, "D": 1.2861, "E": 1.6652}),
}
TOTE_WIDTH = 0.197
POST_MARGIN = 0.15
MIN_GAP = 0.03
MAX_PER_TIER = 3
MIN_PER_SHELF = 2


def _tier_bin_ranges(x_min: float, x_max: float) -> list[tuple[float, float]]:
    usable_min = x_min + POST_MARGIN
    usable_max = x_max - POST_MARGIN
    slot_width = (usable_max - usable_min) / MAX_PER_TIER
    ranges = []
    for i in range(MAX_PER_TIER):
        bin_lo = usable_min + i * slot_width
        bin_hi = bin_lo + slot_width
        ranges.append((bin_lo + TOTE_WIDTH / 2 + MIN_GAP / 2, bin_hi - TOTE_WIDTH / 2 - MIN_GAP / 2))
    return ranges


def sample_stochastic_placements(seed: int | None = None) -> dict[str, dict]:
    """Returns a dict of {unique_label: dict(pos=..., yaw_deg=...)}, one entry
    per sampled tote, following the same occupancy/placement rules as
    ShelfGroupDR: each tier independently holds 0..MAX_PER_TIER totes, each
    shelf resampled until its total >= MIN_PER_SHELF, X placement via fixed
    non-overlapping per-tier bins."""
    rng = random.Random(seed)
    placements: dict[str, dict] = {}
    for shelf_name, shelf in STOCHASTIC_SHELVES.items():
        bin_ranges = _tier_bin_ranges(shelf["x_min"], shelf["x_max"])
        tier_names = list(shelf["tiers"].keys())
        while True:
            occupancy = {t: rng.randint(0, MAX_PER_TIER) for t in tier_names}
            if sum(occupancy.values()) >= MIN_PER_SHELF:
                break
        for tier_name, count in occupancy.items():
            if count == 0:
                continue
            bin_indices = rng.sample(range(MAX_PER_TIER), k=count)
            for n, bin_idx in enumerate(bin_indices):
                lo, hi = bin_ranges[bin_idx]
                x = rng.uniform(lo, hi)
                z = shelf["tiers"][tier_name]
                label = f"{shelf_name}-{tier_name}-{n}"
                placements[label] = dict(pos=(x, shelf["y"], z), yaw_deg=shelf["yaw_deg"])
        print(f"  {shelf_name}: occupancy={occupancy} (total={sum(occupancy.values())})")
    return placements


def _yaw_quat(yaw_deg: float) -> list[float]:
    half = math.radians(yaw_deg) / 2.0
    return [math.cos(half), 0.0, 0.0, math.sin(half)]


def _add_static_body(spec: mujoco.MjSpec, mjcf_dir: Path, label: str) -> None:
    """Mirror MujocoSimulator._build_static_object: one welded body, one mesh
    asset + one geom per collision piece, no free joint."""
    collision_files = sorted(glob.glob(str(mjcf_dir / "collision" / "*.obj")))
    body = spec.worldbody.add_body(name=label, pos=[0, 0, 0], quat=[1, 0, 0, 0])
    for i, f in enumerate(collision_files):
        mesh_name = f"{label}_mesh_convex{i}"
        spec.add_mesh(name=mesh_name, file=f)
        body.add_geom(
            name=f"{label}_convex_{i}",
            meshname=mesh_name,
            type=mujoco.mjtGeom.mjGEOM_MESH,
            condim=4,
            friction=[0.8, 0.05, 0.005],
            rgba=[0.55, 0.4, 0.25, 1.0],
            solref=[0.005, 2],
        )


def _add_free_body(
    spec: mujoco.MjSpec,
    mjcf_dir: Path,
    label: str,
    pos: tuple[float, float, float],
    quat: list[float],
) -> None:
    """Mirror MujocoSimulator._build_object: one free body, mass split evenly
    across convex pieces."""
    collision_files = sorted(glob.glob(str(mjcf_dir / "collision" / "*.obj")))
    num_convex = len(collision_files)
    body = spec.worldbody.add_body(name=label, pos=list(pos), quat=quat)
    body.add_freejoint()
    for i, f in enumerate(collision_files):
        mesh_name = f"{label}_mesh_convex{i}"
        spec.add_mesh(name=mesh_name, file=f)
        body.add_geom(
            name=f"{label}_convex_{i}",
            meshname=mesh_name,
            type=mujoco.mjtGeom.mjGEOM_MESH,
            condim=4,
            mass=0.1 / num_convex,
            friction=[0.8, 0.05, 0.005],
            rgba=[0.9, 0.55, 0.3, 1.0],
            solref=[0.005, 2],
        )


def build_model(placements: dict[str, dict]) -> mujoco.MjModel:
    spec = mujoco.MjSpec()
    spec.option.timestep = 0.01
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 960

    floor = spec.worldbody.add_geom()
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.size = [6.0, 6.0, 0.01]
    floor.name = "ground"
    floor.rgba = [0.5, 0.5, 0.55, 1.0]
    floor.friction = [1.0, 0.005, 0.0001]

    _add_static_body(spec, CORRIDOR_DIR, "corridor0")

    for name, slot in placements.items():
        _add_free_body(
            spec,
            TOTEWEG_DIR,
            f"toteweg_{name}",
            pos=slot["pos"],
            quat=_yaw_quat(slot["yaw_deg"]),
        )

    return spec.compile()


def render_views(model: mujoco.MjModel, data: mujoco.MjData, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    renderer = mujoco.Renderer(model, width=1280, height=960)

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, cam)

    views = {
        "topdown": dict(lookat=[0.0, 0.7, 1.0], distance=6.5, azimuth=90, elevation=-89),
        "oblique": dict(lookat=[0.0, 0.7, 1.0], distance=6.0, azimuth=35, elevation=-25),
        "aisle": dict(lookat=[0.0, 0.7, 1.0], distance=7.5, azimuth=0, elevation=-15),
        "l1_closeup": dict(lookat=[-1.09, 1.80, 1.0], distance=2.8, azimuth=270, elevation=-10),
        "l3_closeup": dict(lookat=[-2.91, 1.80, 1.0], distance=2.8, azimuth=270, elevation=-10),
        "r1_closeup": dict(lookat=[1.13, -0.47, 1.0], distance=3.2, azimuth=270, elevation=-10),
    }
    for name, params in views.items():
        cam.lookat[:] = params["lookat"]
        cam.distance = params["distance"]
        cam.azimuth = params["azimuth"]
        cam.elevation = params["elevation"]
        renderer.update_scene(data, camera=cam)
        img = renderer.render()
        path = out_dir / f"shelf_scenario_{name}.png"
        import imageio.v2 as imageio

        imageio.imwrite(path, img)
        print(f"Saved {path}")

    renderer.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--slots",
        nargs="+",
        default=list(SLOTS.keys()),
        choices=list(SLOTS.keys()),
        help="Which candidate slots to populate with a toteweg crate (default: all 7).",
    )
    parser.add_argument(
        "--stochastic",
        action="store_true",
        help="Use the per-tier stochastic occupancy sampler (ShelfGroupDR-equivalent) "
        "instead of the fixed --slots list.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Seed for --stochastic sampling (default: nondeterministic).",
    )
    parser.add_argument(
        "--settle",
        action="store_true",
        help="Step physics for 2s before rendering, to confirm crates rest stably.",
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Open the interactive MuJoCo passive viewer instead of rendering PNGs.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=REPO_ROOT / "media_gen" / "toteweg_corridor",
        help="Directory for rendered PNGs.",
    )
    args = parser.parse_args()

    if args.stochastic:
        print("Sampling stochastic shelf occupancy:")
        placements = sample_stochastic_placements(seed=args.seed)
        print(f"  total totes spawned: {len(placements)}")
    else:
        placements = {name: SLOTS[name] for name in args.slots}

    model = build_model(placements)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    if args.settle:
        steps = int(2.0 / model.opt.timestep)
        for _ in range(steps):
            mujoco.mj_step(model, data)
        print(f"Settled for {steps} steps ({steps * model.opt.timestep:.2f}s simulated).")
        max_drift = 0.0
        for name in placements:
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"toteweg_{name}")
            target = placements[name]["pos"]
            drift = float(sum((data.xpos[body_id][i] - target[i]) ** 2 for i in range(3)) ** 0.5)
            max_drift = max(max_drift, drift)
        print(f"  max drift from target across {len(placements)} totes: {max_drift * 100:.2f} cm")

    if args.viewer:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("Viewer open. Close the window to exit.")
            while viewer.is_running():
                if args.settle is False:
                    mujoco.mj_step(model, data)
                viewer.sync()
        return

    render_views(model, data, args.out_dir)


if __name__ == "__main__":
    main()
