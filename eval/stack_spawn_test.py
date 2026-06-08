#!/usr/bin/env python3
"""Sequentially spawn stacked MJCF objects in MuJoCo and record a video.

Each copy is released one after another, placed above the current stack with
small incremental rotations.

Install dependencies:
    pip install -r eval/requirements.txt
"""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import imageio.v2 as imageio
import mujoco


FLOOR_GEOM_NAME = "drop_floor"
HIDE_Z = 50.0
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
        description="Sequentially spawn stacked MJCF objects in MuJoCo.",
        epilog=(
            "Examples:\n"
            "  python eval/stack_spawn_test.py --xml eval/output/smoke_complete.xml\n"
            "  python eval/stack_spawn_test.py --xml assets/bin_b04/MJCF/bin_b04.xml "
            "--count 4 --spawn-interval 1.5 --rotation-step 12"
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
        help="Path to MJCF/XML file to replicate in the scene.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=3,
        help="Number of object copies to spawn sequentially.",
    )
    parser.add_argument(
        "--spawn-interval",
        type=float,
        default=1.0,
        help="Seconds between each spawn event.",
    )
    parser.add_argument(
        "--base-height",
        type=float,
        default=1.0,
        help="Bottom height in meters for the first spawned object.",
    )
    parser.add_argument(
        "--stack-gap",
        type=float,
        default=0.05,
        help="Vertical gap in meters between stacked spawn positions.",
    )
    parser.add_argument(
        "--rotation-step",
        type=float,
        default=12.0,
        help="Yaw increment in degrees applied per spawn index.",
    )
    parser.add_argument(
        "--tilt-step",
        type=float,
        default=4.0,
        help="Pitch increment in degrees applied per spawn index.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=8.0,
        help="Simulation duration in seconds.",
    )
    parser.add_argument("--fps", type=int, default=30, help="Video frame rate.")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help=(
            "Output video path (.mp4 or .gif). "
            "Defaults to eval/output/<xml_stem>_stack_spawn.mp4."
        ),
    )
    parser.add_argument(
        "--root-body",
        type=str,
        default=None,
        help=(
            "Name of the top-level body in the source XML. "
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
            repo_root / "eval" / "output" / f"{args.xml_path.stem}_stack_spawn.mp4"
        )
    return args


def _count_geoms_in_spec(spec: mujoco.MjSpec) -> int:
    return sum(len(list(body.geoms)) for body in spec.bodies)


def _resolve_root_body_name(spec: mujoco.MjSpec, requested_name: Optional[str]) -> str:
    world_children = list(spec.worldbody.bodies)
    child_names = [body.name for body in world_children]

    if requested_name is not None:
        if spec.body(requested_name) is None:
            raise ValueError(
                f"Body '{requested_name}' not found. "
                f"Top-level bodies present: {child_names or ['none']}"
            )
        return requested_name

    for name in PREFERRED_ROOT_BODY_NAMES:
        if name in child_names:
            return name

    if len(world_children) == 1:
        return world_children[0].name

    raise ValueError(
        "Could not determine which body to replicate. "
        f"Top-level bodies present: {child_names or ['none']}. "
        "Pass --root-body explicitly."
    )


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


def _object_z_bounds(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    root_body_id: int,
    floor_geom_id: int,
) -> tuple[float, float]:
    object_bodies = _find_descendants(model, root_body_id)
    object_geom_ids = [
        geom_id
        for geom_id in range(model.ngeom)
        if geom_id != floor_geom_id and int(model.geom_bodyid[geom_id]) in object_bodies
    ]
    if not object_geom_ids:
        raise ValueError(f"No geoms found under body id {root_body_id}.")

    lowest_z = min(
        float(data.geom_xpos[geom_id, 2] - model.geom_rbound[geom_id])
        for geom_id in object_geom_ids
    )
    highest_z = max(
        float(data.geom_xpos[geom_id, 2] + model.geom_rbound[geom_id])
        for geom_id in object_geom_ids
    )
    return lowest_z, highest_z


def _quat_from_axis_angle(axis: tuple[float, float, float], angle_rad: float) -> list[float]:
    ax, ay, az = axis
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm == 0.0:
        return [1.0, 0.0, 0.0, 0.0]
    ax, ay, az = ax / norm, ay / norm, az / norm
    half = angle_rad / 2.0
    s = math.sin(half)
    return [math.cos(half), ax * s, ay * s, az * s]


def _quat_mul(
    q1: tuple[float, float, float, float],
    q2: tuple[float, float, float, float],
) -> list[float]:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]


def _spawn_rotation(
    index: int,
    rotation_step_deg: float,
    tilt_step_deg: float,
) -> list[float]:
    yaw = _quat_from_axis_angle((0.0, 0.0, 1.0), math.radians(rotation_step_deg * index))
    pitch = _quat_from_axis_angle((1.0, 0.0, 0.0), math.radians(tilt_step_deg * index))
    return _quat_mul(yaw, pitch)


@dataclass
class SpawnInstance:
    index: int
    body_name: str
    body_id: int
    qpos_adr: int
    dof_adr: int
    spawned: bool = False


@dataclass
class StackLayout:
    object_height: float
    spawn_bottom_heights: list[float]


def _prepare_object_spec(mjcf_path: Path, root_body_name: str) -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(mjcf_path))
    if _count_geoms_in_spec(spec) == 0:
        raise ValueError(
            "The MJCF appears incomplete (0 geoms). Generate a populated MJCF first."
        )
    root = spec.body(root_body_name)
    if root is None:
        raise ValueError(f"Missing body '{root_body_name}' in {mjcf_path}.")
    if len(list(root.joints)) == 0:
        root.add_freejoint()
    return spec


def build_model(
    mjcf_path: Path,
    count: int,
    base_height: float,
    stack_gap: float,
    root_body_name: Optional[str] = None,
) -> tuple[mujoco.MjModel, list[SpawnInstance], int, StackLayout]:
    source = mujoco.MjSpec.from_file(str(mjcf_path))
    resolved_root_name = _resolve_root_body_name(source, root_body_name)

    scene = mujoco.MjSpec()
    floor = scene.worldbody.add_geom()
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.size = [2.0, 2.0, 0.01]
    floor.name = FLOOR_GEOM_NAME
    floor.rgba = [0.6, 0.6, 0.6, 1.0]

    for index in range(count):
        instance = _prepare_object_spec(mjcf_path, resolved_root_name)
        frame = scene.worldbody.add_frame()
        scene.attach(instance, prefix=f"obj{index}_", frame=frame)

    model = scene.compile()
    floor_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, FLOOR_GEOM_NAME)
    if floor_geom_id < 0:
        raise RuntimeError("Failed to add floor geom for stack spawn evaluation.")

    instances: list[SpawnInstance] = []
    for index in range(count):
        body_name = f"obj{index}_{resolved_root_name}"
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            raise ValueError(f"Compiled model is missing body '{body_name}'.")

        first_joint = int(model.body_jntadr[body_id])
        joint_count = int(model.body_jntnum[body_id])
        free_joint = None
        for joint_id in range(first_joint, first_joint + joint_count):
            if int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_FREE):
                free_joint = joint_id
                break
        if free_joint is None:
            raise ValueError(f"Body '{body_name}' does not have a free joint.")

        instances.append(
            SpawnInstance(
                index=index,
                body_name=body_name,
                body_id=body_id,
                qpos_adr=int(model.jnt_qposadr[free_joint]),
                dof_adr=int(model.jnt_dofadr[free_joint]),
            )
        )

    object_height = _measure_object_height(model, instances[0].body_id, floor_geom_id)
    stack_layout = _compute_stack_layout(object_height, count, base_height, stack_gap)
    return model, instances, floor_geom_id, stack_layout


def _measure_object_height(
    model: mujoco.MjModel,
    body_id: int,
    floor_geom_id: int,
) -> float:
    data = mujoco.MjData(model)
    first_joint = int(model.body_jntadr[body_id])
    if int(model.body_jntnum[body_id]) == 0:
        raise ValueError(f"Body id {body_id} has no joint for height measurement.")
    qpos_adr = int(model.jnt_qposadr[first_joint])
    data.qpos[qpos_adr : qpos_adr + 3] = [0.0, 0.0, 0.0]
    data.qpos[qpos_adr + 3 : qpos_adr + 7] = [1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)
    lowest_z, highest_z = _object_z_bounds(model, data, body_id, floor_geom_id)
    return highest_z - lowest_z


def _compute_stack_layout(
    object_height: float,
    count: int,
    base_height: float,
    stack_gap: float,
) -> StackLayout:
    spawn_bottom_heights = [
        base_height + index * (object_height + stack_gap) for index in range(count)
    ]
    return StackLayout(
        object_height=object_height,
        spawn_bottom_heights=spawn_bottom_heights,
    )


def _hide_instance(data: mujoco.MjData, instance: SpawnInstance) -> None:
    adr = instance.qpos_adr
    data.qpos[adr : adr + 3] = [float(instance.index) * 0.25, 0.0, HIDE_Z]
    data.qpos[adr + 3 : adr + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[instance.dof_adr : instance.dof_adr + 6] = 0.0


def _spawn_instance(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    instance: SpawnInstance,
    floor_geom_id: int,
    target_bottom_z: float,
    rotation_step_deg: float,
    tilt_step_deg: float,
) -> None:
    adr = instance.qpos_adr
    data.qpos[adr : adr + 3] = [0.0, 0.0, 0.0]
    data.qpos[adr + 3 : adr + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[instance.dof_adr : instance.dof_adr + 6] = 0.0
    mujoco.mj_forward(model, data)

    lowest_z, _ = _object_z_bounds(model, data, instance.body_id, floor_geom_id)
    data.qpos[adr + 2] += target_bottom_z - lowest_z
    rotation = _spawn_rotation(instance.index, rotation_step_deg, tilt_step_deg)
    data.qpos[adr + 3 : adr + 7] = rotation
    data.qvel[instance.dof_adr : instance.dof_adr + 6] = 0.0
    mujoco.mj_forward(model, data)
    instance.spawned = True


def run_stack_spawn_test(
    model: mujoco.MjModel,
    instances: list[SpawnInstance],
    floor_geom_id: int,
    stack_layout: StackLayout,
    spawn_interval: float,
    rotation_step_deg: float,
    tilt_step_deg: float,
    duration: float,
    fps: int,
    output_path: Path,
    use_viewer: bool,
) -> None:
    data = mujoco.MjData(model)
    for instance in instances:
        _hide_instance(data, instance)
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
            "Failed to initialize MuJoCo renderer. " + backend_hint
        ) from exc

    frames: list = []
    step_count = 0
    next_spawn_idx = 0
    spawn_times = [index * spawn_interval for index in range(len(instances))]

    viewer_ctx = None
    if use_viewer:
        viewer_ctx = mujoco.viewer.launch_passive(model, data)

    try:
        while data.time < duration:
            while (
                next_spawn_idx < len(instances)
                and data.time + 1e-9 >= spawn_times[next_spawn_idx]
            ):
                instance = instances[next_spawn_idx]
                target_bottom_z = stack_layout.spawn_bottom_heights[next_spawn_idx]

                _spawn_instance(
                    model=model,
                    data=data,
                    instance=instance,
                    floor_geom_id=floor_geom_id,
                    target_bottom_z=target_bottom_z,
                    rotation_step_deg=rotation_step_deg,
                    tilt_step_deg=tilt_step_deg,
                )
                print(
                    f"Spawned {instance.body_name} at t={data.time:.3f}s, "
                    f"bottom z={target_bottom_z:.4f} m"
                )
                next_spawn_idx += 1

            if step_count % frame_interval == 0:
                renderer.update_scene(data)
                frames.append(renderer.render())

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
    print(f"Saved stack spawn video: {output_path}")
    print(f"Duration simulated: {data.time:.3f}s")
    print(f"Object height: {stack_layout.object_height:.4f} m")
    print(f"Objects spawned: {sum(1 for instance in instances if instance.spawned)}")


def main() -> None:
    args = parse_args()
    if not args.xml_path.is_file():
        raise FileNotFoundError(f"MJCF/XML file not found: {args.xml_path}")
    if args.output.suffix.lower() not in {".mp4", ".gif"}:
        raise ValueError("--output must end with .mp4 or .gif")
    if args.count <= 0:
        raise ValueError("--count must be > 0")
    if args.spawn_interval < 0:
        raise ValueError("--spawn-interval must be >= 0")
    if args.duration <= 0:
        raise ValueError("--duration must be > 0")
    if args.fps <= 0:
        raise ValueError("--fps must be > 0")
    if args.stack_gap < 0:
        raise ValueError("--stack-gap must be >= 0")

    model, instances, floor_geom_id, stack_layout = build_model(
        args.xml_path,
        args.count,
        args.base_height,
        args.stack_gap,
        args.root_body,
    )
    run_stack_spawn_test(
        model=model,
        instances=instances,
        floor_geom_id=floor_geom_id,
        stack_layout=stack_layout,
        spawn_interval=args.spawn_interval,
        rotation_step_deg=args.rotation_step,
        tilt_step_deg=args.tilt_step,
        duration=args.duration,
        fps=args.fps,
        output_path=args.output,
        use_viewer=args.viewer,
    )


if __name__ == "__main__":
    main()
