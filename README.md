# USD2MJCF
A powerful tool for converting Universal Scene Description (USD) files to MuJoCo XML Configuration Format (MJCF).

Welcome to the Lightwheel open-source community!

Join us, contribute, and help shape the future of AI and robotics. For questions or collaboration, contact Frank Chen at ming.chen@lightwheel.ai.

## 🌐Overview
USD2MJCF is a powerful tool that converts Universal Scene Description (USD) assets into MuJoCo's MJCF format, enabling seamless transitions from collaborative 3D workflows to high-fidelity physics simulation.

## 📚Documentation
See [docs/README.md](docs/README.md) for a full USD-to-MJCF guide, including key concepts, collision development, a `bin_b04` walkthrough, and evaluation workflows.

## 🚀Features
 - Accurate material conversion from USD to MJCF format
 - Mesh segmentation by material ID for better asset organization
 - Correct joint conversion, preserving articulation structure
 - Automated collision shape generation and splitting
 - Precise 3D positioning to maintain spatial consistency

## 🛠️ Setup
### Prerequisites
- Python 3.10
- pip package manager
- [Miniconda](https://docs.conda.io/en/latest/miniconda.html) or [Anaconda](https://www.anaconda.com/)

### Installation
1. **Clone this repository**:  
   replace `$REPO_ROOT` with the path you want
    ```bash
    git clone <repository-url> $REPO_ROOT
    ```
2. **Install required dependencies**:  
   replace `$YOUR_ENV_NAME` with the name you want
    ```bash
    conda create --name $YOUR_ENV_NAME python=3.10
    conda activate $YOUR_ENV_NAME
    cd $REPO_ROOT
    pip install -r requirements.txt
    ```
## 🚀Getting started

```bash
cd $REPO_ROOT
python3 test/usd2mjcf_test.py $USD_FILE_PATH [--output_path=$OUTPUT_DIRECTORY] [--generate_collision [--preprocess_resolution=20] [--resolution=2000]] [--resolve_external_assets|--no-resolve_external_assets] [--asset_cache_dir=$CACHE_DIR] [--resolver_strict]
```

### 📋 Command Line Parameters

| Parameter | Parameter Type | Data Type | Default | Description |
|-----------|----------------|-----------|---------|-------------|
| `input_path` | Required | String | - | Path to the input USD file to convert |
| `--output_path` | Optional | String | Same as input | Directory where MJCF files will be saved |
| `--generate_collision` | Flag | Boolean | False | Generate collision meshes using convex decomposition. USD collision bodies can be non-convex, but MJCF collision bodies must be convex. Since collision bodies are not exported during conversion, this option recreates them from visual meshes. |
| `--preprocess_resolution` | Optional | Integer | 20 | Preprocessing voxelization resolution for convex decomposition |
| `--resolution` | Optional | Integer | 2000 | Main voxelization resolution for convex decomposition |
| `--resolve_external_assets` / `--no-resolve_external_assets` | Optional | Boolean | True | Resolve external USD refs (e.g., HTTPS payloads and `file:/isaac-sim/...`) into a local cache for plain Python runtimes |
| `--asset_cache_dir` | Optional | String | `assets/_resolved_cache` | Directory used to cache mirrored external assets and patched input USD files |
| `--resolver_strict` | Flag | Boolean | False | Fail conversion if any external reference cannot be resolved |

### 💡 Usage Examples

**Basic conversion (visual only):**
```bash
python3 test/usd2mjcf_test.py /path/to/robot.usd
```

**Convert with collision generation:**
```bash
python3 test/usd2mjcf_test.py /path/to/robot.usd --generate_collision
```

**High-quality collision with custom resolution:**
```bash
python3 test/usd2mjcf_test.py /path/to/robot.usd --generate_collision --preprocess_resolution=40 --resolution=4000
```

**Specify custom output directory:**
```bash
python3 test/usd2mjcf_test.py /path/to/robot.usd --output_path=/custom/output/dir --generate_collision
```

**Convert with external dependency resolving (recommended for Omniverse-linked USDs):**
```bash
python3 test/usd2mjcf_test.py assets/bin_b04.usda --generate_collision --resolve_external_assets
```

**Use strict resolver mode and custom cache directory:**
```bash
python3 test/usd2mjcf_test.py assets/bin_b04.usda --resolve_external_assets --asset_cache_dir=/tmp/usd_cache --resolver_strict
```

​If **`--output_path`** is not provided, the converter will automatically create an output folder in the same directory as the input USD file.​​

## 🗂️ Batch Processing

For processing multiple USD files at once, you can use the batch conversion script:

```bash
cd $REPO_ROOT
python3 test/batch_convert.py $INPUT_PATH [--generate_collision [--preprocess_resolution=20] [--resolution=2000]] [--resolve_external_assets|--no-resolve_external_assets] [--asset_cache_dir=$CACHE_DIR] [--resolver_strict]
```

### 📋 Batch Processing Parameters

| Parameter | Parameter Type | Data Type | Default | Description |
|-----------|----------------|-----------|---------|-------------|
| `input_path` | Required | String | - | Path to input USD file or directory containing USD files |
| `--generate_collision` | Flag | Boolean | False | Generate collision meshes using convex decomposition |
| `--preprocess_resolution` | Optional | Integer | 20 | Preprocessing voxelization resolution for convex decomposition |
| `--resolution` | Optional | Integer | 2000 | Main voxelization resolution for convex decomposition |
| `--resolve_external_assets` / `--no-resolve_external_assets` | Optional | Boolean | True | Resolve external USD refs into local cache before conversion |
| `--asset_cache_dir` | Optional | String | `assets/_resolved_cache` | Shared cache directory for mirrored external assets |
| `--resolver_strict` | Flag | Boolean | False | Fail on unresolved external references |

### 💡 Batch Processing Examples

**Convert all USD files in a directory (visual only):**
```bash
python3 test/batch_convert.py /path/to/usd/directory/
```

**Convert all USD files with collision generation:**
```bash
python3 test/batch_convert.py /path/to/usd/directory/ --generate_collision
```

**Batch convert with custom collision resolution:**
```bash
python3 test/batch_convert.py /path/to/usd/directory/ --generate_collision --preprocess_resolution=40 --resolution=4000
```

**Batch convert a single file:**
```bash
python3 test/batch_convert.py /path/to/robot.usd --generate_collision
```

**Batch convert with resolver enabled (default):**
```bash
python3 test/batch_convert.py /path/to/usd/directory/ --generate_collision --resolve_external_assets
```

**Note:** The batch converter will recursively process all **`.usd`** and **`.usda`** files in the specified directory and automatically skip temporary files (files containing **`.tmp.usd`**). Each file will be converted and saved to the same directory as the USD file.​​

### External Asset Resolver Notes

- The resolver mirrors remote external dependencies (such as HTTPS payloads) into a local cache.
- Cached files are reused across runs to avoid repeated downloads.
- A patched temporary input USD is generated only when external refs need rewriting.
- Resolver summary is printed during conversion (`downloaded`, `reused`, `rewritten`, `unresolved`).

## 🔖Version
Current version: 1.0.0

## 🤝Support
For issues and questions, please contact the development team or create an issue in the repository.

## Acknowledgements
This project is based on NVIDIA's pip package nvidia-srl-usd-to-urdf


