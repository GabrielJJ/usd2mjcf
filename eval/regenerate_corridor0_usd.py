#!/usr/bin/env python3
"""Regenerate `corridor0.usd` from the current (post Row-swap) visual mesh.

Context (see docs/teleop_simple_study/toteweg_factory_scene_migration_plan.md,
section "7a) Row swap", and the investigation this script came out of): the
Row swap rewrote `MJCF/visuals/corridor0.obj` and the per-shelf collision
`.obj`s in place with a rigid transform (l1/l3 <-> r1/r2/r3 blocks swapped
between Y-bands, each shelf also rotated 180 deg), and updated
`src/simple/dr/shelf_group.py`'s `SHELF_SPECS` to match -- but never
regenerated `corridor0.usd`/`corridor0_light.usd` (the files Isaac Sim
actually renders), because `pxr` was unavailable in that authoring
environment. Confirmed directly (not assumed): `md5sum` of the current
`corridor0.usd` is byte-identical to `corridor0_legacy/corridor0.usd` (the
deliberately-kept pre-swap rollback copy).

This script closes that gap. It needs only `trimesh` + `pxr` (`usd-core`,
already in this venv's requirements.txt) -- no GPU, no Isaac Sim install.

Usage (from repo root, using this submodule's own venv which already has
trimesh+usd-core installed):
    third_party/usd2mjcf/.venv/bin/python3 \\
        third_party/usd2mjcf/eval/regenerate_corridor0_usd.py [--check-only]

--check-only: only run the AABB comparison (no file writes) -- useful to
re-verify alignment after the fact, or to check drift without touching the
binaries.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import trimesh
from pxr import Gf, Usd, UsdGeom

REPO_ROOT = Path(__file__).resolve().parents[3]

# The staged, source-of-truth corridor0 asset (what FixturesAssetManager.load
# resolves at runtime, src/simple/assets/fixtures.py) plus the third_party
# converter-bundle copy that's kept in sync with it by repo convention (see
# "Files modified for the swap" in the migration plan doc, section 7a).
TARGETS = [
    dict(
        obj=REPO_ROOT / "src/simple/assets/fixtures/corridor0/MJCF/visuals/corridor0.obj",
        usd=REPO_ROOT / "src/simple/assets/fixtures/corridor0/corridor0.usd",
        collision_glob=REPO_ROOT / "src/simple/assets/fixtures/corridor0/MJCF/collision",
    ),
    dict(
        obj=REPO_ROOT / "third_party/usd2mjcf/assets/corridor0/MJCF/visuals/visual.obj",
        usd=REPO_ROOT / "third_party/usd2mjcf/assets/corridor0/corridor0_light.usd",
        collision_glob=REPO_ROOT / "third_party/usd2mjcf/assets/corridor0/MJCF/collision",
    ),
]

# Prim structure of the *current* (stale, pre-swap) corridor0.usd, confirmed
# by opening it directly with pxr in this investigation:
#   /Root (Xform, default prim)
#     /Root/corridor0 (Xform)
# The regenerated stage reproduces this exact layout so
# IsaacSimSimulator.__create_object/__update_object (src/simple/engines/isaacsim.py)
# -- which reference the whole .usd via add_reference_to_stage and never look
# up a specific mesh sub-path -- keep working unmodified.
ROOT_PATH = "/Root"
CORRIDOR_XFORM_PATH = "/Root/corridor0"
MESH_PATH = "/Root/corridor0/Mesh"


def _combined_collision_bounds(collision_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    mins, maxs = [], []
    for obj_path in sorted(collision_dir.glob("*.obj")):
        mesh = trimesh.load(obj_path, process=False)
        mins.append(mesh.bounds[0])
        maxs.append(mesh.bounds[1])
    return np.min(mins, axis=0), np.max(maxs, axis=0)


def _usd_bounds(usd_path: Path) -> tuple[np.ndarray, np.ndarray]:
    stage = Usd.Stage.Open(str(usd_path))
    prim = stage.GetDefaultPrim()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return np.array(rng.GetMin()), np.array(rng.GetMax())


def _report_alignment(label: str, obj_path: Path, usd_path: Path, collision_dir: Path) -> None:
    obj_mesh = trimesh.load(obj_path, process=False)
    obj_min, obj_max = obj_mesh.bounds[0], obj_mesh.bounds[1]
    col_min, col_max = _combined_collision_bounds(collision_dir)
    usd_min, usd_max = _usd_bounds(usd_path)

    print(f"\n=== {label} ===")
    print(f"  visual .obj AABB : min={obj_min}  max={obj_max}")
    print(f"  collision AABB   : min={col_min}  max={col_max}")
    print(f"  usd AABB         : min={usd_min}  max={usd_max}")
    print(f"  usd-vs-obj delta : min={usd_min - obj_min}  max={usd_max - obj_max}")

    delta = np.abs(usd_min - obj_min).max()
    status = "OK (aligned)" if delta < 1e-3 else f"MISALIGNED (max |delta|={delta:.4f} m)"
    print(f"  -> {status}")


def regenerate(obj_path: Path, usd_path: Path) -> None:
    mesh = trimesh.load(obj_path, process=False)
    print(f"Loaded {obj_path}: {len(mesh.vertices)} verts, {len(mesh.faces)} faces")

    stage = Usd.Stage.CreateNew(str(usd_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    root_xform = UsdGeom.Xform.Define(stage, ROOT_PATH)
    stage.SetDefaultPrim(root_xform.GetPrim())

    UsdGeom.Xform.Define(stage, CORRIDOR_XFORM_PATH)

    usd_mesh = UsdGeom.Mesh.Define(stage, MESH_PATH)
    usd_mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in mesh.vertices])
    usd_mesh.CreateFaceVertexCountsAttr([3] * len(mesh.faces))
    usd_mesh.CreateFaceVertexIndicesAttr(mesh.faces.flatten().tolist())
    if mesh.vertex_normals is not None and len(mesh.vertex_normals) == len(mesh.vertices):
        usd_mesh.CreateNormalsAttr([Gf.Vec3f(*n) for n in mesh.vertex_normals])
        usd_mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    usd_mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)

    stage.GetRootLayer().Save()
    print(f"Wrote {usd_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-only", action="store_true", help="Only report AABB alignment, don't write files")
    args = parser.parse_args()

    for target in TARGETS:
        if not args.check_only:
            regenerate(target["obj"], target["usd"])
        _report_alignment(target["usd"].parent.name, target["obj"], target["usd"], target["collision_glob"])


if __name__ == "__main__":
    main()
