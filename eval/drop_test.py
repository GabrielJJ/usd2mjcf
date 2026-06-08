#!/usr/bin/env python3
"""Drop-test an MJCF object in MuJoCo and record a video.

Prerequisite for this repository's sample asset:
    python test/usd2mjcf_test.py assets/bin_b04.usda --generate_collision

Install dependencies:
    pip install -r eval/requirements.txt
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Optional

import imageio.v2 as imageio
import mujoco


FLOOR_GEOM_NAME = "drop_floor"
PREFERRED_ROOT_BODY_NAMES = ("Root", "RootNode")


def _resolve_xml_path(
    positional: Optional[Path],
    flag: Optional[Path],
    default: Path,
) -> Path:
    if positional is not None and flag is not None and positional != flag:
        raise ValueError(
            f"Conflicting XML paths: positional {positional} != --xml {flag}"
        )
    return flag or positional or default


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    default_xml = repo_root / "assets" / "MJCF" / "bin_b04.xml"

    parser = argparse.ArgumentParser(
        description="Run a MuJoCo drop evaluation on an MJCF/XML model.",
        epilog=(
            "Examples:\n"
            "  python eval/drop_test.py --xml assets/MJCF/bin_b04.xml\n"
            "  python eval/drop_test.py eval/output/smoke_complete.xml --output eval/output/smoke_drop.gif"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "xml",
        nargs="?",
        type=Path,
        default=None,
        help="Path to MJCF/XML file (alternative to --xml).",
    )
    parser.add_argument(
        "--xml",
        dest="xml_flag",
        type=Path,
        default=None,
        help="Path to MJCF/XML file to drop-test.",
    )
    parser.add_argument(
        "--mjcf",
        dest="xml_flag",
        type=Path,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--height",
        type=float,
        default=1.0,
        help="Target spawn height in meters above ground.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="Simulation duration in seconds.",
    )
    parser.add_argument("--fps", type=int, default=30, help="Video frame rate.")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output video path (.mp4 or .gif). Defaults to eval/output/<xml_stem>_drop.mp4.",
    )
    parser.add_argument(
        "--root-body",
        type=str,
        default=None,
        help=(
            "Name of the top-level body to drop. "
            "Auto-detected from worldbody children when omitted."
        ),
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Also open the MuJoCo passive viewer while simulating.",
    )
    args = parser.parse_args()
    args.xml_path = _resolve_xml_path(args.xml, args.xml_flag, default_xml)
    if args.output is None:
        args.output = (
            repo_root / "eval" / "output" / f"{args.xml_path.stem}_drop.mp4"
        )
    return args


def _count_geoms_in_spec(spec: mujoco.MjSpec) -> int:
    return sum(len(list(body.geoms)) for body in spec.bodies)


def _find_descendants(model: mujoco.MjModel, root_body_id: int) -> set[int]:
    descendants = {root_body_id}
    changed = True
    while changed:
        changed = False
        for body_id in range(model.nbody):
            parent_id = int(model.body_parentid[body_id])
            if parent_id in descendants and body_id not in descendants:
                descendants.add(body_id)
                changed = True
    return descendants


def _first_free_joint_on_body(model: mujoco.MjModel, body_id: int) -> Optional[int]:
    first_joint = int(model.body_jntadr[body_id])
    joint_count = int(model.body_jntnum[body_id])
    for joint_id in range(first_joint, first_joint + joint_count):
        if int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_FREE):
            return joint_id
    return None


def _resolve_root_body(spec: mujoco.MjSpec, requested_name: Optional[str]):
    world_children = list(spec.worldbody.bodies)
    child_names = [body.name for body in world_children]

    if requested_name is not None:
        root = spec.body(requested_name)
        if root is None:
            raise ValueError(
                f"Body '{requested_name}' not found. "
                f"Top-level bodies present: {child_names or ['none']}"
            )
        return root, requested_name

    for name in PREFERRED_ROOT_BODY_NAMES:
        if name in child_names:
            return spec.body(name), name

    if len(world_children) == 1:
        body = world_children[0]
        return body, body.name

    raise ValueError(
        "Could not determine which body to drop. "
        f"Top-level bodies present: {child_names or ['none']}. "
        "Pass --root-body explicitly."
    )


def build_model(
    mjcf_path: Path,
    spawn_height: float,
    root_body_name: Optional[str] = None,
) -> tuple[mujoco.MjModel, int, int, str]:
    spec = mujoco.MjSpec.from_file(str(mjcf_path))
    if _count_geoms_in_spec(spec) == 0:
        raise ValueError(
            "The MJCF appears incomplete (0 geoms). Generate a populated MJCF first with:\n"
            "python test/usd2mjcf_test.py assets/bin_b04.usda --generate_collision"
        )

    root, resolved_root_name = _resolve_root_body(spec, root_body_name)

    if len(list(root.joints)) == 0:
        root.add_freejoint()
    root.pos[2] = spawn_height

    floor = spec.worldbody.add_geom()
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.size = [2.0, 2.0, 0.01]
    floor.name = FLOOR_GEOM_NAME
    floor.rgba = [0.6, 0.6, 0.6, 1.0]

    model = spec.compile()
    root_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, resolved_root_name
    )
    floor_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, FLOOR_GEOM_NAME)
    if root_body_id < 0:
        raise ValueError(f"Compiled model is missing body '{resolved_root_name}'.")
    if floor_geom_id < 0:
        raise RuntimeError("Failed to add floor geom for drop evaluation.")
    return model, root_body_id, floor_geom_id, resolved_root_name


def run_drop_test(
    model: mujoco.MjModel,
    root_body_id: int,
    root_body_name: str,
    floor_geom_id: int,
    spawn_height: float,
    duration: float,
    fps: int,
    output_path: Path,
    use_viewer: bool,
) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    root_free_joint = _first_free_joint_on_body(model, root_body_id)
    if root_free_joint is None:
        raise ValueError(
            f"Body '{root_body_name}' does not have a free joint, cannot run drop test."
        )

    qpos_adr = int(model.jnt_qposadr[root_free_joint])
    dof_adr = int(model.jnt_dofadr[root_free_joint])

    # Align so the lowest object point starts exactly at requested height.
    object_bodies = _find_descendants(model, root_body_id)
    object_geom_ids = [
        geom_id
        for geom_id in range(model.ngeom)
        if geom_id != floor_geom_id and int(model.geom_bodyid[geom_id]) in object_bodies
    ]
    if not object_geom_ids:
        raise ValueError(
            f"No object geoms found under '{root_body_name}' body after compilation."
        )

    lowest_z = min(
        float(data.geom_xpos[geom_id, 2] - model.geom_rbound[geom_id])
        for geom_id in object_geom_ids
    )
    data.qpos[qpos_adr + 2] += spawn_height - lowest_z
    mujoco.mj_forward(model, data)

    dt = float(model.opt.timestep)
    frame_interval = max(1, round(1.0 / max(fps, 1) / dt))
    output_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        renderer = mujoco.Renderer(model, width=640, height=480)
    except mujoco.FatalError as exc:
        backend_hint = (
            "Set MUJOCO_GL to an offscreen backend (for example `egl` or `osmesa`) "
            "when running headless."
        )
        current_backend = os.environ.get("MUJOCO_GL")
        if current_backend:
            backend_hint = (
                f"Current MUJOCO_GL={current_backend!r} failed. Try another backend "
                "such as `egl` or `osmesa`."
            )
        raise RuntimeError(
            "Failed to initialize MuJoCo renderer. "
            + backend_hint
        ) from exc
    frames = []
    first_contact_time = None
    max_downward_vel = 0.0
    step_count = 0

    viewer_ctx = None
    if use_viewer:
        viewer_ctx = mujoco.viewer.launch_passive(model, data)

    try:
        while data.time < duration:
            if step_count % frame_interval == 0:
                renderer.update_scene(data)
                frames.append(renderer.render())

            if first_contact_time is None:
                for c_idx in range(data.ncon):
                    contact = data.contact[c_idx]
                    if (
                        int(contact.geom1) == floor_geom_id
                        or int(contact.geom2) == floor_geom_id
                    ):
                        first_contact_time = float(data.time)
                        break

            max_downward_vel = min(max_downward_vel, float(data.qvel[dof_adr + 2]))

            mujoco.mj_step(model, data)
            step_count += 1
            if viewer_ctx is not None:
                viewer_ctx.sync()

        renderer.update_scene(data)
        frames.append(renderer.render())
    finally:
        if viewer_ctx is not None:
            viewer_ctx.close()
        renderer.close()

    imageio.mimsave(output_path, frames, fps=fps)
    final_z = float(data.xpos[root_body_id, 2])
    print(f"Saved drop video: {output_path}")
    print(f"Duration simulated: {data.time:.3f}s")
    print(
        "First contact time: "
        + ("none" if first_contact_time is None else f"{first_contact_time:.3f}s")
    )
    print(f"Final {root_body_name} body z: {final_z:.4f} m")
    print(f"Max downward velocity: {max_downward_vel:.4f} m/s")


def main() -> None:
    args = parse_args()
    if not args.xml_path.is_file():
        raise FileNotFoundError(f"MJCF/XML file not found: {args.xml_path}")
    output_suffix = args.output.suffix.lower()
    if output_suffix not in {".mp4", ".gif"}:
        raise ValueError("--output must end with .mp4 or .gif")
    if args.duration <= 0:
        raise ValueError("--duration must be > 0")
    if args.fps <= 0:
        raise ValueError("--fps must be > 0")

    model, root_body_id, floor_geom_id, root_body_name = build_model(
        args.xml_path,
        args.height,
        args.root_body,
    )
    run_drop_test(
        model=model,
        root_body_id=root_body_id,
        root_body_name=root_body_name,
        floor_geom_id=floor_geom_id,
        spawn_height=args.height,
        duration=args.duration,
        fps=args.fps,
        output_path=args.output,
        use_viewer=args.viewer,
    )


if __name__ == "__main__":
    main()
