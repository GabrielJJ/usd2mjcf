import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lightwheel.srl.from_usd.to_mjcf import UsdToMjcf
from utils.resolve_usd_assets import resolve_usd_input

def main():
    parser = argparse.ArgumentParser(description="Convert USD to MJCF")
    parser.add_argument("input_path", type=str, help="Path to input USD file")
    parser.add_argument("--output_path", type=str, default=None, help="Path to output MJCF file")
    parser.add_argument("--generate_collision", action='store_true', help="Generate collision meshes for the MJCF model")
    parser.add_argument("--preprocess_resolution", type=int, default=20, help="Preprocessing voxelization resolution for convex decomposition")
    parser.add_argument("--resolution", type=int, default=2000, help="Main voxelization resolution for convex decomposition")
    parser.add_argument(
        "--resolve_external_assets",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Resolve external USD refs (https/file) into a local cache.",
    )
    parser.add_argument(
        "--asset_cache_dir",
        type=str,
        default=None,
        help="Directory to cache resolved external assets.",
    )
    parser.add_argument(
        "--resolver_strict",
        action="store_true",
        help="Fail when an external reference cannot be resolved.",
    )
    
    args = parser.parse_args()

    input_path = Path(args.input_path)
    output_path = args.output_path
    generate_collision = args.generate_collision
    preprocess_resolution = args.preprocess_resolution
    resolution = args.resolution
    cache_dir = Path(args.asset_cache_dir) if args.asset_cache_dir else Path(__file__).resolve().parents[1] / "assets" / "_resolved_cache"
    
    if output_path is None:
        output_path = str(input_path.parent)
    mjcf_dir = os.path.join(output_path, 'MJCF')

    resolved_input, summary = resolve_usd_input(
        input_path,
        cache_dir,
        enabled=args.resolve_external_assets,
        strict=args.resolver_strict,
    )
    if args.resolve_external_assets:
        print(
            "[resolver] downloaded={d} reused={r} rewritten={w} unresolved={u}".format(
                d=summary.downloaded,
                r=summary.reused,
                w=summary.rewritten,
                u=summary.unresolved,
            )
        )
        if summary.patched_file is not None:
            print(f"[resolver] patched_input={summary.patched_file}")
    
    mjcf_file = UsdToMjcf.init_from_file(str(resolved_input)).save_to_file(mjcf_dir)
    
    # Only generate collision meshes if requested
    if generate_collision:
        from utils.add_collision import add_collision_to_only_visual_mjcf
        add_collision_to_only_visual_mjcf(mjcf_file, preprocess_resolution=preprocess_resolution, resolution=resolution)

if __name__ == "__main__":
    main()


