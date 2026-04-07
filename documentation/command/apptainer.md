# Apptainer / HPC Workflow

DNA provides a macOS-compatible workflow for deploying slurm jobs on HPC servers that use
[Apptainer](https://apptainer.org/) (e.g., Valeria, Compute Canada / Digital Research Alliance of Canada).

> ⚠️ **Apptainer is Linux-only and is NOT supported on macOS.**
> DNA's role is to **locally build** the slurm Docker image and then either:
> - **Save** it as a `linux/amd64` Docker tar archive (`.tar`) to be transferred to the HPC server, OR
> - **Push** it to a Docker registry from which the HPC server can pull it.
> DNA **never executes `apptainer` locally**.

## Overview

DNA supports two distinct use cases for running Apptainer slurm jobs on an HPC server.
Both share the same build step and HPC profile configuration, but differ in how the job is submitted.

In both use cases, `dna build slurm --apptainer <profile>` requires choosing a **pipeline** via
`--save` (tar archive) or `--push` (Docker registry):

| Pipeline flag | What it does |
|---|---|
| `--save` | Saves the slurm image as a `.tar` archive and generates `dna_tar_to_apptainer_sif_converter.sh` and `dna_hpc_server_config.bash` |
| `--push` | Pushes the slurm image to a Docker registry and generates `dna_registry_to_apptainer_sif_converter.sh` and `dna_hpc_server_config.bash` |


### Use Case 1 — SBATCH Template Workflow

Submit jobs using the `slurm_job.<DNA_SJOB_NAME>.apptainer.<profile>.bash` script (copied and renamed
from `slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.<profile>.bash` added by `dna init`). You edit the
copy directly (set `DNA_SJOB_NAME`, `python_arguments`, callbacks) and submit it with `sbatch`. This is
the primary workflow for recurring, configurable jobs.

**Tar archive pipeline (`--save`):**
```
 [Local]  1. dna build slurm --apptainer <profile> --save
             → builds Docker image, saves linux/amd64 tar archive (.tar),
               generates dna_tar_to_apptainer_sif_converter.sh and dna_hpc_server_config.bash
 [Local]  2. Copy slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.<profile>.bash → slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.<profile>.bash
             → set DNA_SJOB_NAME, python_arguments, and optional callbacks
 [Local]  3. Transfer to HPC (use your preferred method, e.g., rsync, scp, sftp):
               artifact/apptainer/, slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.<profile>.bash,
               .dockerized_norlab/,
               data/external_data/, data/repository_data/
               (data/shared_data/ is optional)
 [HPC]    4. bash artifact/apptainer/dna_hpc_server_config.bash   ← first time only
             → sets up project directory structure, loads apptainer, authenticates with Docker registry
 [HPC]    5. bash artifact/apptainer/dna_tar_to_apptainer_sif_converter.sh
             → converts tar archive to SIF image
 [HPC]    6. from super-project root dir execute $ sbatch slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.<profile>.bash
```

**Registry push pipeline (`--push`):**
```
 [Local]  1. dna build slurm --apptainer <profile> --push
             → builds Docker image, pushes to Docker registry (requires authentication),
               generates dna_registry_to_apptainer_sif_converter.sh and dna_hpc_server_config.bash
 [Local]  2. Copy and edit slurm job template (same as above)
 [HPC]    3. Transfer artifact/apptainer/ to HPC
 [HPC]    4. bash artifact/apptainer/dna_hpc_server_config.bash   ← first time only
             → sets up project directory structure, loads apptainer, authenticates with Docker registry
 [HPC]    5. bash artifact/apptainer/dna_registry_to_apptainer_sif_converter.sh
             → pulls image from registry, converts to SIF
 [HPC]    6. from super-project root dir execute $ sbatch slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.<profile>.bash
```

### Use Case 2 — Generated Script Workflow

Generate a standalone run script locally via `dna run slurm <sjob-id> --ga <profile> -- <args>`,
transfer it, and run it directly on the HPC server. Python arguments are pre-baked into the script
from the CLI. No SLURM directives or callbacks. Useful for quick one-off runs or CI pipelines.

**Tar archive pipeline (`--save`):**
```
 [Local]  1. dna build slurm --apptainer <profile> --save
             → builds Docker image, saves linux/amd64 tar archive (.tar),
               generates dna_tar_to_apptainer_sif_converter.sh and dna_hpc_server_config.bash
 [Local]  2. dna run slurm <sjob-id> --ga <profile> -- <args>
             → generates artifact/apptainer/run_apptainer_<sjob-id>.sh (does NOT execute)
 [Local]  3. Transfer to HPC (use your preferred method, e.g., rsync, scp, sftp):
               artifact/apptainer/,
               .dockerized_norlab/,
               data/external_data/, data/repository_data/
               (data/shared_data/ is optional)
 [HPC]    4. bash artifact/apptainer/dna_hpc_server_config.bash   ← first time only
             → sets up project directory structure, loads apptainer, authenticates with Docker registry
 [HPC]    5. bash artifact/apptainer/dna_tar_to_apptainer_sif_converter.sh
             → converts tar archive to SIF image
 [HPC]    6. bash artifact/apptainer/run_apptainer_<sjob-id>.sh
```

**Registry push pipeline (`--push`):**
```
 [Local]  1. dna build slurm --apptainer <profile> --push
             → builds Docker image, pushes to registry,
               generates dna_registry_to_apptainer_sif_converter.sh and dna_hpc_server_config.bash
 [Local]  2. dna run slurm <sjob-id> --ga <profile> -- <args>
             → generates artifact/apptainer/run_apptainer_<sjob-id>.sh (does NOT execute)
 [HPC]    3. Transfer artifact/apptainer/ to HPC
 [HPC]    4. bash artifact/apptainer/dna_hpc_server_config.bash   ← first time only
             → sets up project directory structure, loads apptainer, authenticates with Docker registry
 [HPC]    5. bash artifact/apptainer/dna_registry_to_apptainer_sif_converter.sh
             → pulls image from registry, converts to SIF
 [HPC]    6. bash artifact/apptainer/run_apptainer_<sjob-id>.sh
```

## Supported HPC Server Profiles

| Profile | Server | Runtime | DNA on server? |
|---------|--------|---------|----------------|
| `valeria` | Ulaval Valeria | Apptainer only | ❌ Standalone scripts |
| `compute_canada` | Compute Canada / Digital Research Alliance | Apptainer only | ❌ Standalone scripts |
| `mamba` | NorLab Mamba (Apptainer workflow) | Docker or Apptainer | ❌ Standalone scripts |
| _(default)_ | NorLab Mamba (Docker workflow) | Docker | ✅ `dna run slurm` |

> **Note:** Mamba supports **both** Docker and Apptainer workflows.
> Use the Docker workflow (`dna run slurm`) when DNA is installed on Mamba.
> Use the Apptainer workflow (`--apptainer mamba`) to build locally and deploy standalone scripts — DNA is not required on Mamba for this path.

## Prerequisites

| Location | Requirement |
|----------|-------------|
| Local (macOS) | Docker, DNA installed |
| HPC server | **Apptainer ≥ 1.1.0** installed, project files transferred |

> ℹ️ Apptainer ≥ 1.1.0 is required for `--no-eval`, `--cleanenv`, and `--env-file` comment
> handling. Generated scripts and slurm job templates print an informational warning at startup.

## Quick Start

Both use cases share a common prerequisite: configure your HPC server profile.

### Step 0 — Configure your HPC server profile

HPC server profile files are copied to your super project by `dna init`. If you need to
add a profile manually:

```bash
# For Valeria:
cp src/lib/template/.dockerized_norlab/configuration/hpc_server_profile/.env.valeria \
   .dockerized_norlab/configuration/hpc_server_profile/.env.valeria

# For Compute Canada:
cp src/lib/template/.dockerized_norlab/configuration/hpc_server_profile/.env.compute_canada \
   .dockerized_norlab/configuration/hpc_server_profile/.env.compute_canada

# For Mamba (Apptainer workflow — Mamba also supports Docker workflow via dna run slurm):
cp src/lib/template/.dockerized_norlab/configuration/hpc_server_profile/.env.mamba \
   .dockerized_norlab/configuration/hpc_server_profile/.env.mamba
```

`DN_PROJECT_PATH` is automatically set by `dna init` using the resolved `DN_PROJECT_GIT_NAME`
from the super project's git remote URL. No manual editing is required for this field.

---

## Use Case 1 — SBATCH Template Workflow

Use the `slurm_job.<DNA_SJOB_NAME>.apptainer.<profile>.bash` script (copied and renamed from
`slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.<profile>.bash` added by `dna init`) to submit recurring,
configurable jobs via `sbatch`. You edit `DNA_SJOB_NAME`, `python_arguments`, and the optional
setup/teardown callbacks directly in the script.

### Step 1 — Build locally (cross-platform for HPC target)

**Tar archive pipeline (`--save`):**
```bash
dna build slurm --apptainer valeria --save
```

This:
- Builds the slurm Docker image targeting `linux/amd64` (from `APPTAINER_TARGET_PLATFORM` in profile)
- Saves it as a `linux/amd64` tar archive (`.tar`) to `artifact/apptainer/`
- Generates `artifact/apptainer/dna_tar_to_apptainer_sif_converter.sh` (run on HPC to convert `.tar` → SIF)
- Generates `artifact/apptainer/dna_hpc_server_config.bash` (run once on HPC to set up directory structure and authenticate with Docker registry)

**Registry push pipeline (`--push`):**
```bash
dna build slurm --apptainer valeria --push
```

This:
- Builds the slurm Docker image
- Pushes it to the Docker registry (requires Docker Hub authentication: `docker login`)
- Generates `artifact/apptainer/dna_registry_to_apptainer_sif_converter.sh` (run on HPC to pull from registry → SIF)
- Generates `artifact/apptainer/dna_hpc_server_config.bash` (run once on HPC to set up directory structure and authenticate with Docker registry)

> 💡 **Tip: Use `--squash` to reduce image size before saving/pushing.**
>
> ```bash
> dna build slurm --apptainer valeria --save --squash
> dna build slurm --apptainer valeria --push --squash
> ```
>
> This collapses all Docker image layers into a single layer, reducing tar archive or registry push size.
>
> ℹ️ **Note:** Squashing collapses all Docker image layers into a single flat layer.
> All metadata (ENV, ENTRYPOINT, CMD, WORKDIR, USER, LABEL) is preserved.
> The image cannot be used for further incremental builds.

> 💡 **Tip: Use `--docker-login` when authenticating with the push pipeline on the HPC server.**
>
> When running `dna_registry_to_apptainer_sif_converter.sh` on the HPC server for a private
> registry image, pass `--docker-login` to authenticate interactively:
>
> ```bash
> bash artifact/apptainer/dna_registry_to_apptainer_sif_converter.sh --docker-login
> ```
>
> Apptainer will prompt for your credentials once and cache them for the session.

> 💡 **Tip: Use `--gs-only` to re-generate the converter script without rebuilding the image.**
>
> If you only need to update the generated HPC converter script (e.g., after a DNA upgrade that
> changes the generated script logic) without re-building or re-saving/pushing the Docker image,
> use `--gs-only` together with `--apptainer <profile>` and `--save` or `--push`:
>
> ```bash
> # Re-generate dna_tar_to_apptainer_sif_converter.sh only:
> dna build slurm --apptainer valeria --save --gs-only
>
> # Re-generate dna_registry_to_apptainer_sif_converter.sh only:
> dna build slurm --apptainer valeria --push --gs-only
> ```
>
> This flag is exclusive to the `--apptainer` workflow and does **not** require internet access.

### Step 2 — Edit the slurm job template

Copy and rename the template, then edit it locally:
```bash
cp slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.<profile>.bash slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.<profile>.bash
```
Edit `slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.<profile>.bash`:
- Set `DNA_SJOB_NAME` (recommend using an issue tracker ID)
- Set `python_arguments` (your Python module and its arguments)
- Optionally update `job_setup_callback()` / `job_teardown_callback()`

### Step 3 — Transfer to HPC

Transfer the following files/directories to your super-project root on the HPC server using your preferred method (e.g., rsync, scp, sftp):
- `artifact/apptainer/` — tar archive + `dna_tar_to_apptainer_sif_converter.sh` + `dna_hpc_server_config.bash`
- `slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.<profile>.bash` — the sbatch script
- `.dockerized_norlab/` — DNA configuration directory
- `data/external_data/` — non-tracked external data (if applicable)
- `data/repository_data/` — data required by src/test code logic (if applicable)
- `data/shared_data/` — optional; may be replaced by a local data volume on the HPC server

### Step 4 — Configure HPC server (first time only)

Run this script **once** on the HPC server after first cloning or transferring the super-project.
It sets up the directory structure, loads the Apptainer module, and authenticates with Docker registry:

```bash
# On the HPC server (from super-project root):
bash artifact/apptainer/dna_hpc_server_config.bash
```

This script:
1. Creates all expected super-project directories (`artifact/apptainer/`, `artifact/optuna_storage/`, `artifact/slurm_jobs_logs/`, `artifact/tensorboard_tmp/`, `data/external_data/`, `data/repository_data/`, `data/shared_data/`, `slurm_jobs/`).
2. Loads Apptainer via `module load apptainer/<latest-version>` (tries the highest available version via `module spider`, falls back to the default).
3. Prompts for your Docker Hub username and runs `apptainer registry login --username <username> docker://docker.io` interactively. No secrets are stored by DNA.

> 💡 **Tip: Use `--target-dir` to specify a custom HPC super-project root.**
>
> Pass `--target-dir` to both `dna_hpc_server_config.bash` and the converter scripts when the
> super-project lives at a non-standard path (e.g., scratch filesystem):
>
> ```bash
> bash artifact/apptainer/dna_hpc_server_config.bash --target-dir /scratch/myproject
> bash artifact/apptainer/dna_tar_to_apptainer_sif_converter.sh --target-dir /scratch/myproject
> ```

### Step 5 — Build SIF on HPC server

```bash
# On the HPC server (from super-project root):
bash artifact/apptainer/dna_tar_to_apptainer_sif_converter.sh
```

This script:
1. Converts the Docker tar archive to a SIF image using `apptainer build`.
2. **Automatically deletes the tar archive** after successful conversion to free disk space.

> ⚠️ **Note:** If `apptainer build` fails, the script exits immediately and **preserves the tar archive** so you can retry the conversion.

### Step 6 — Submit the slurm job

```bash
# On the HPC server (from super-project root):
sbatch slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.<profile>.bash
```

---

## Use Case 2 — Generated Script Workflow

Use `dna run slurm <sjob-id> --ga <profile> -- <args>` to generate a standalone run script
locally with python arguments pre-baked from the CLI. Transfer and run it directly on the
HPC server. No `#SBATCH` directives or callbacks. Useful for quick one-off runs or CI pipelines.

### Step 1 — Build locally (cross-platform for HPC target)

Same as Use Case 1 Step 1 — choose the `--save` or `--push` pipeline. See that section for details, including the `--squash` and `--gs-only` tips.

### Step 2 — Generate the Apptainer run script

```bash
dna run slurm <sjob-id> --generate-apptainer valeria -- launcher/train.py --epochs=10
# Or using shorthand:
dna run slurm <sjob-id> --ga valeria -- launcher/train.py --epochs=10
```

This **generates** (does NOT execute) a standalone `artifact/apptainer/run_apptainer_<sjob-id>.sh`
script for the HPC server. Use `--print-only` to print the command instead of writing a file:

```bash
dna run slurm <sjob-id> --ga valeria --print-only -- launcher/train.py
```

### Step 3 — Transfer to HPC

Transfer the following files/directories to your super-project root on the HPC server using your preferred method (e.g., rsync, scp, sftp):
- `artifact/apptainer/` — tar archive + `dna_tar_to_apptainer_sif_converter.sh` + `dna_hpc_server_config.bash` + generated run script
- `.dockerized_norlab/` — DNA configuration directory
- `data/external_data/` — non-tracked external data (if applicable)
- `data/repository_data/` — data required by src/test code logic (if applicable)
- `data/shared_data/` — optional; may be replaced by a local data volume on the HPC server

### Step 4 — Configure HPC server (first time only)

Same as Use Case 1 Step 4. See that section for details.

### Step 5 — Build SIF on HPC server

```bash
# On the HPC server (from super-project root):
bash artifact/apptainer/dna_tar_to_apptainer_sif_converter.sh
```

Same behaviour as Use Case 1 Step 5.

> 💡 **Tip: Use `--target-dir`** — same as Use Case 1. Pass the same path to both `dna_hpc_server_config.bash` and `dna_tar_to_apptainer_sif_converter.sh`.

### Step 6 — Run the generated script

```bash
# On the HPC server:
bash artifact/apptainer/run_apptainer_<sjob-id>.sh
```

## Understanding the Apptainer Pipeline Artifacts

The Apptainer slurm pipeline involves **four related but distinct artifacts**, all sharing
the same HPC profile dotenv configuration:

```
HPC Profile Dotenv (.env.<profile>)
        │
        ├──── sourced by ──── Slurm Job Script (slurm_job.<DNA_SJOB_NAME>.apptainer.<profile>.bash)
        │                         │
        │                         └── user edits TODO markers, submits via: sbatch slurm_job.<DNA_SJOB_NAME>.apptainer.<profile>.bash
        │
        └──── sourced by ──── Generated Run Script (run_apptainer_<dna_sjob_name>.sh)
                                  │
                                  └── auto-generated by: dna run slurm <sjob-id> --ga <profile> -- <args>
```

### 1. HPC Profile Dotenv Files (shared config)

**Location (in super project):** `.dockerized_norlab/configuration/hpc_server_profile/.env.<profile>`

Configuration files that define HPC-server-specific environment variables (`DN_PROJECT_USER`,
`DN_PROJECT_PATH`, etc.). Used **twice**: locally by DNA at build time (to bake `DN_PROJECT_USER`
into the Docker image) and on the HPC server at runtime (sourced by both the slurm job templates
and generated run scripts).

### 2. Slurm Job Scripts (user-editable sbatch scripts)

**Template location (in super project):** `slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.<profile>.bash`
**Working copy location:** `slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.<profile>.bash` (user-renamed copy)

Standalone SLURM sbatch scripts added to the user's project by `dna init` under `slurm_jobs/template/`.
They include `#SBATCH` directives, `job_setup_callback()` / `job_teardown_callback()` hooks, and `TODO`
markers for `DNA_SJOB_NAME` and `python_arguments`. The user copies and renames the template, edits it, and
submits via `sbatch`. These are the **primary way to submit jobs** on the HPC server.

### 3. `dna run slurm --ga` Output (auto-generated run scripts)

**Location:** Generated at `artifact/apptainer/run_apptainer_<dna_sjob_name>.sh`

Simpler, auto-generated scripts produced by `dna run slurm <sjob-id> --ga <profile> -- <args>`.
They source the same HPC profile dotenv and use the same `apptainer exec` flags, but have
python args pre-baked from the CLI and no SLURM directives or callbacks. Useful for quick
one-off runs or CI pipelines.

| Aspect | Slurm Job Template | `--ga` Generated Script |
|---|---|---|
| **Purpose** | Full sbatch job with SLURM directives | Minimal run-only script |
| **Customization** | User edits `DNA_SJOB_NAME`, `python_arguments`, callbacks | Pre-baked from CLI args |
| **SLURM directives** | Yes (`#SBATCH --gres`, `--time`, etc.) | No |
| **Setup/teardown hooks** | Yes | No |
| **How to run** | `sbatch slurm_job.<DNA_SJOB_NAME>.apptainer.<profile>.bash` | `bash run_apptainer_<dna_sjob_name>.sh` |
| **Created by** | `dna init` (copied to project) | `dna run slurm --ga` (generated on demand) |

## CLI Reference

### `dna build slurm --apptainer <profile> --save|--push`

```bash
# Tar archive pipeline (--save):
dna build slurm --apptainer <profile> --save [--squash]

# Registry push pipeline (--push) — requires Docker Hub authentication:
dna build slurm --apptainer <profile> --push [--squash]

# Re-generate converter script only (skip docker build/push/save):
dna build slurm --apptainer <profile> --save --gs-only
dna build slurm --apptainer <profile> --push --gs-only
```

| Option | Description |
|--------|-------------|
| `--apptainer <profile>` | **Required.** HPC Apptainer workflow for slurm service. Must be combined with `--save` or `--push`. |
| `--save` | Tar archive pipeline: saves slurm image as `linux/amd64` `.tar` archive to `artifact/apptainer/`. Generates `dna_tar_to_apptainer_sif_converter.sh` and `dna_hpc_server_config.bash`. |
| `--push` | Registry pipeline: pushes slurm image to Docker registry. Generates `dna_registry_to_apptainer_sif_converter.sh` and `dna_hpc_server_config.bash`. Requires `docker login`. |
| `--squash` | Squash slurm image layers before saving/pushing. Reduces size. See [Squashing note](#squash-note). |
| `--gs-only` | **(Apptainer-only)** Skip docker build/push/save and re-generate only the HPC converter script. **Requires `--apptainer <profile>` and `--save` or `--push`**. Useful to update the converter script without rebuilding the image. Does **not** require internet. |

**`--save` pipeline output files in `artifact/apptainer/`:**
- `<project>-slurm.<tag>.tar` — Docker tar archive
- `dna_tar_to_apptainer_sif_converter.sh` — Helper script to run on HPC (converts tar → SIF)
- `dna_hpc_server_config.bash` — One-time HPC setup script (directory structure + apptainer auth)

**`--push` pipeline output files in `artifact/apptainer/`:**
- `dna_registry_to_apptainer_sif_converter.sh` — Helper script to run on HPC (pulls from registry → SIF)
- `dna_hpc_server_config.bash` — One-time HPC setup script (directory structure + apptainer auth)

#### `dna_hpc_server_config.bash` Options

```bash
bash artifact/apptainer/dna_hpc_server_config.bash [--target-dir <TARGET-DIRECTORY-PATH>] [--help]
```

| Option | Description |
|--------|-------------|
| `--target-dir <PATH>` | Optional. Path to the HPC super-project root. When set, creates the directory structure under `<PATH>`. Defaults to two levels above the script location. |
| `--help` | Print usage and exit. |

This script (run **once** per HPC server setup): creates all 8 required super-project directories + conditionally runs `module load apptainer/<latest>` (tries highest version via `module spider`, falls back to default) + prompts for Docker Hub username and runs `apptainer registry login --username <username> docker://docker.io` interactively (no secrets stored by DNA).

#### `dna_tar_to_apptainer_sif_converter.sh` Options

```bash
bash artifact/apptainer/dna_tar_to_apptainer_sif_converter.sh [--target-dir <TARGET-DIRECTORY-PATH>] [--help]
```

| Option | Description |
|--------|-------------|
| `--target-dir <PATH>` | Optional. Path to the HPC super-project root. When set, outputs the SIF to `<PATH>/artifact/apptainer/`. Defaults to two levels above the script location (i.e., inferred from the standard `artifact/apptainer/` placement). |
| `--help` | Print usage and exit. |

This script: conditionally runs `module load apptainer/<latest>` (tries highest version via `module spider`, falls back to default) + sets `APPTAINER_CACHEDIR`/`APPTAINER_TMPDIR` via `mktemp -d -p "${SLURM_TMPDIR}"` (uses SLURM node scratch when available, works on all HPC servers) + conditionally applies `--mksquashfs-args="-comp zstd -Xcompression-level 19"` when apptainer ≥ 1.4.0 (skips flag for older versions) + builds the SIF to a staging area first, then moves to final destination + deletes the tar after successful conversion (preserved on failure).

#### `dna_registry_to_apptainer_sif_converter.sh` Options

```bash
bash artifact/apptainer/dna_registry_to_apptainer_sif_converter.sh [OPTIONS]
```

| Option | Description |
|--------|-------------|
| `--target-dir <PATH>` | Optional. Path to the HPC super-project root. When set, outputs the SIF to `<PATH>/artifact/apptainer/`. |
| `--docker-login` | Optional. Authenticate interactively with docker.io before building. Apptainer prompts for credentials once. Use this for private registry images. |
| `--help` | Print usage and exit. |

This script: conditionally runs `module load apptainer/<latest>` (tries highest version via `module spider`, falls back to default) + sets `APPTAINER_CACHEDIR`/`APPTAINER_TMPDIR` via `mktemp -d -p "${SLURM_TMPDIR}"` (uses SLURM node scratch when available, works on all HPC servers) + optionally authenticates via `--docker-login` + conditionally applies `--mksquashfs-args="-comp zstd -Xcompression-level 19"` when apptainer ≥ 1.4.0 + builds the SIF to a staging area first, then moves to final destination.

### `dna save --apptainer <profile> DIRPATH slurm`

```bash
dna save --apptainer <profile> DIRPATH slurm [--squash]
```

| Option | Description |
|--------|-------------|
| `--apptainer <profile>` | Generate Apptainer tar archive artifacts alongside the saved image. |
| `--squash` | Squash slurm image before saving the tar archive. Reduces transfer size. See [Squashing note](#squash-note). |

<a name="squash-note"></a>
> ℹ️ **Squashing note:** `--squash` collapses all Docker image layers into a single flat layer.
> All metadata (ENV, ENTRYPOINT, CMD, WORKDIR, USER, LABEL) is preserved.
> The image cannot be used for further incremental builds.

### `dna run slurm <sjob-id> --generate-apptainer <profile> [OPTIONS] [--] <python-args>`

```bash
dna run slurm <sjob-id> --generate-apptainer <profile> [OPTIONS] [--] <python-args>
dna run slurm <sjob-id> --ga <profile> [OPTIONS] [--] <python-args>
```

| Option | Description |
|--------|-------------|
| `--generate-apptainer`, `--ga` `<profile>` | Route to Apptainer workflow (generates script, does NOT run apptainer locally). See `dna run --help-slurm-apptainer`. |
| `--sif-path <path>` | Path to the SIF file on the HPC server (default: `artifact/apptainer/<image>-slurm.sif`) |
| `--output-dir <path>` | Directory for generated scripts (default: `artifact/apptainer/`) |
| `--print-only` | Print apptainer exec command to stdout only (do not write script file) |
| `--log-name <name>` | Log file name (for script header comment) |

## HPC Server Profile Configuration

Profile env files serve a dual purpose:
1. **At local build time** — DNA sources the profile to read `DN_PROJECT_USER` so the Docker
   image is built with the correct HPC username (avoiding Apptainer UID/GID mismatches).
2. **On the HPC server** — the profile is sourced by the generated run scripts to configure
   runtime paths, cache directories, and entrypoint options.

| Variable | Used at | Description |
|----------|---------|-------------|
| `DN_PROJECT_USER` | Build time | **Required.** HPC server username — baked into the Docker image so the container user matches the Apptainer host user. |
| `DN_PROJECT_PATH` | Runtime | Path to the project **inside the container** (auto-set by `dna init` from `DN_PROJECT_GIT_NAME`) |
| `DN_HOST` | Build time | Target platform (`linux/x86`) |
| `APPTAINER_TARGET_PLATFORM` | Build time | Docker build/save platform (default: `linux/amd64`). Sets `DOCKER_DEFAULT_PLATFORM` during `dna build slurm --apptainer` to enforce cross-architecture builds on Apple Silicon Macs. Also passed as `--platform` to `docker image save` (for `--save` pipeline) to ensure the exported tar archive targets the correct architecture. |

> **Note (`APPTAINER_CACHEDIR`/`APPTAINER_TMPDIR` handling — all HPC profiles):**
>
> - **Setup script** (`dna_hpc_server_config.bash`) and **converter scripts** (`dna_tar_to_apptainer_sif_converter.sh`, `dna_registry_to_apptainer_sif_converter.sh`):
>   All three generated scripts use a unified, HPC-server-agnostic tmpdir mechanism:
>   ```bash
>   export APPTAINER_CACHEDIR="$( mktemp -d -p "${SLURM_TMPDIR}" 2>/dev/null || mktemp -d )"
>   export APPTAINER_TMPDIR="$( mktemp -d -p "${SLURM_TMPDIR}" 2>/dev/null || mktemp -d )"
>   ```
>   When `SLURM_TMPDIR` is set (i.e., inside an sbatch job), the temp directories are allocated on
>   the fast local node scratch filesystem. When `SLURM_TMPDIR` is not set (interactive login nodes),
>   `mktemp -d` falls back to the system default. The SIF is always built to this staging area first,
>   then `mv`'d to the final destination to avoid Lustre home quota issues.
>   **For best results, run the converter scripts via sbatch** (not interactively on the login node)
>   so that `SLURM_TMPDIR` points to local node scratch.
>
> - **Slurm job execution templates** (`slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash`, `slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash`, `slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash`):
>   All three templates use the same unified mechanism for `APPTAINER_CACHEDIR` and `APPTAINER_TMPDIR`
>   as the converter and HPC config scripts:
>   ```bash
>   export APPTAINER_CACHEDIR="$( mktemp -d -p "${SLURM_TMPDIR}" 2>/dev/null || mktemp -d )"
>   export APPTAINER_TMPDIR="$( mktemp -d -p "${SLURM_TMPDIR}" 2>/dev/null || mktemp -d )"
>   ```
>   This ensures Apptainer's cache (OCI layers, pulled images) is stored on local node scratch and
>   never lands in the Lustre home directory (quota-limited, no atomic rename). Consistent with
>   Valeria best practices (https://doc.s3.valeria.science/fr/calcul/apptainer.html#bonnes-pratiques) and
>   applicable to all three Apptainer HPC profiles (valeria, compute_canada, mamba).

## Slurm Job Templates

Templates are located in `slurm_jobs/template/` in the super project (added by `dna init`).
Copy and rename to `slurm_jobs/slurm_job.<DNA_SJOB_NAME>.apptainer.<profile>.bash` before editing.

| Template file | Profile | Description |
|---------------|---------|-------------|
| `slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash` | `valeria` | Valeria HPC standalone job |
| `slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash` | `compute_canada` | Compute Canada standalone job |
| `slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash` | `mamba` | Mamba HPC standalone job (Apptainer workflow) |

All templates are **standalone** — they do not require DNA on the HPC server.
They source the same HPC profile dotenv file (`.env.<profile>`) as the `--ga` generated scripts
and use the same `apptainer exec` flags. See [Understanding the Apptainer Pipeline Artifacts](#understanding-the-apptainer-pipeline-artifacts) for details.

## Apptainer Exec Flags

DNA generates `apptainer exec` commands with the following hardening flags (based on the
[Apptainer docs](https://apptainer.org/docs/user/latest/docker_and_oci.html)):

| Flag | Purpose |
|------|--------|
| `--no-eval` | Disables shell evaluation of environment variables, matching Docker/OCI behavior. Prevents `$(...)` and backtick expansion in `ENV` values. |
| `--cleanenv` | Blocks host environment variables from leaking into the container (e.g., `PYTHONPATH`, `LD_LIBRARY_PATH` from HPC module systems). Static config is passed via `--env-file`. |
| `--no-home` | Prevents `$HOME` auto-mount, avoiding conflicts with `pip install --user` packages on the HPC host (`~/.local/lib/python*/`). |
| `--nv` | Enables NVIDIA GPU access inside the container (equivalent to Docker's `runtime: nvidia`). Remove this flag for CPU-only jobs. |
| `--env-file` | Passes static environment variables from the HPC profile dotenv file. |
| `--env` | Passes dynamic SLURM runtime variables: `CUDA_VISIBLE_DEVICES`, `SLURM_JOB_ID`, `SLURM_TMPDIR`, `SLURM_JOB_NAME`, `SLURM_NODELIST`. |
| `--writable-tmpfs` | Creates a temporary writable overlay for SIF (read-only SquashFS). |
| `--bind` | Bind-mounts host directories into the container (volumes from docker-compose, minus X11). |
| `--pwd` | Sets container working directory. |

> **Environment variable strategy:** Static configuration (from `docker-compose.run.slurm.yaml`
> `environment:` block) is consolidated in the HPC profile dotenv file and passed via `--env-file`.
> Only truly dynamic SLURM-assigned variables (not known at config time) use `--env` flags.
> If you need `$HOME` access inside the container, add `--bind $HOME:$HOME` in the slurm job template's
> `job_setup_callback` or customize the `apptainer exec` command.

## Docker ↔ Apptainer Feature Mapping

| Docker Compose | Apptainer | Notes |
|----------------|-----------|-------|
| `image:` | SIF from `apptainer build ... docker-archive:<tar>` | Built from DNA `.tar` archive (converted by `dna_tar_to_apptainer_sif_converter.sh`) |
| `volumes:` | `--bind /host:/container[:ro\|:rw]` | Direct mapping |
| `environment:` | `--env-file` (static) + `--env` (dynamic SLURM) | Two-tier strategy |
| `runtime: nvidia` | `--nv` | GPU support (remove for CPU-only jobs) |
| `network_mode: host` | Default in Apptainer | No flag needed |
| `pid: host` | Default in Apptainer | No flag needed |
| `ipc: host` | Default in Apptainer | No flag needed |
| `security_opt: seccomp=unconfined` | Not needed | Apptainer default |
| `WORKDIR` | `--pwd /path` | Explicit flag |
| `USER` | Host UID/GID (automatic) | Apptainer maps calling user |
| `/tmp/numba_cache` writes | `--writable-tmpfs` | SIF is read-only |
| _(no equivalent)_ | `--no-eval` | Prevent ENV shell evaluation |
| _(no equivalent)_ | `--cleanenv` | Prevent host env leakage |
| _(no equivalent)_ | `--no-home` | Prevent `$HOME` auto-mount |

## Known Limitations and Workarounds

### SIF is read-only
Apptainer SIF files are immutable. Directories needing writes must be bind-mounted:
- Use `--bind` for `artifact/`, `data/external_data/`, etc.
- Use `--writable-tmpfs` for `/tmp` writes (numba cache, etc.)

### `DN_PROJECT_USER` / User namespace
Apptainer runs as the calling user (maps host UID/GID). To avoid user mismatch errors,
`DN_PROJECT_USER` must be set in the HPC profile env file (e.g., `.env.valeria`, `.env.compute_canada`, `.env.mamba`) to your
HPC server username **before building** (use command `$ id -un` on hpc server if you are not sure). DNA reads this value at build time and bakes it
into the Docker image. The `--apptainer <profile>` flag on `dna build slurm` and
`dna save` automatically sources the profile and validates `DN_PROJECT_USER`.

If `DN_PROJECT_USER` is missing or set to `PLACEHOLDER_HPC_USERNAME`, the build will fail
with an explicit error message.

### X11 / Display
X11 bind mounts (`/tmp/.X11-unix`) are excluded from HPC Apptainer jobs — HPC nodes
typically have no display server.

### `cap_add: SYS_PTRACE, SYS_NICE`
These capabilities may require HPC admin to configure `allow setuid = yes` in
`apptainer.conf`. Check with your HPC support team.

## Testing

### Containerized Apptainer Integration Tests

DNA includes a containerized test suite that validates the full Apptainer pipeline
using real Apptainer inside a Docker container (Apptainer-in-Docker). This enables
end-to-end testing on macOS without requiring Apptainer installed locally.

```bash
# Run all containerized Apptainer tests
bash tests/tests_containerized_apptainer/run_containerized_apptainer_tests.bash
```

The test suite validates:
- Full `tar → SIF` conversion via `dna_tar_to_apptainer_sif_converter.sh`
- All DNA Apptainer exec flags (`--cleanenv`, `--no-eval`, `--no-home`, `--env-file`, etc.)
- Generated run scripts from `dna run slurm --ga`
- All `slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.*.bash` scripts (valeria, compute_canada, mamba)
- Entrypoint runtime detection (`DNA_RUNTIME=apptainer`)

See `tests/tests_containerized_apptainer/README.md` for details.
