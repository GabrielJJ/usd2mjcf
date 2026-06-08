import argparse
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert USD to MJCF")
    parser.add_argument("input_path", type=str, help="Path to input USD file")
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
    input_path = args.input_path
    generate_collision = args.generate_collision
    preprocess_resolution = args.preprocess_resolution
    resolution = args.resolution
    resolve_external_assets = args.resolve_external_assets
    asset_cache_dir = args.asset_cache_dir
    resolver_strict = args.resolver_strict

    common_args = []
    if generate_collision:
        common_args += [
            "--generate_collision",
            f"--preprocess_resolution={preprocess_resolution}",
            f"--resolution={resolution}",
        ]
    if not resolve_external_assets:
        common_args += ["--no-resolve_external_assets"]
    if asset_cache_dir:
        common_args += [f"--asset_cache_dir={asset_cache_dir}"]
    if resolver_strict:
        common_args += ["--resolver_strict"]

    converter_script = Path(__file__).resolve().parents[0] / "usd2mjcf_test.py"
    python_exe = sys.executable

    if os.path.isdir(input_path):
        for root, _, files in os.walk(input_path):
            for file in files:
                if ".tmp.usd" in file:
                    continue
                if file.endswith(".usd") or file.endswith(".usda"):
                    usd_path = os.path.join(root, file)
                    cmd = [python_exe, str(converter_script), usd_path, *common_args]
                    subprocess.run(cmd, check=False)
    else:
        cmd = [python_exe, str(converter_script), input_path, *common_args]
        subprocess.run(cmd, check=False)