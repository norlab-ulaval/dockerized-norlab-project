# Apptainer / HPC Workflow

DNA provides a macOS-compatible workflow for deploying slurm jobs on HPC servers that use
[Apptainer](https://apptainer.org/) (e.g., Valeria, Compute Canada / Digital Research Alliance of Canada).

> ⚠️ **Apptainer is Linux-only and is NOT supported on macOS.**
> DNA's role is to **locally build and save** a `linux/amd64` Docker tar archive, which is
> then transferred to the HPC server where Apptainer converts and runs it.
> DNA **never executes `apptainer` locally**.

## Overview

```
Local (macOS)                          HPC Server (Linux)
─────────────────────────────────      ──────────────────────────────────
dna build slurm --apptainer <profile>  →  (transfer tar + build_sif.sh)
dna run slurm --ga <profile>            →  (transfer run script)
                                           bash build_sif.sh
                                           sbatch slurm_job.apptainer.<profile>.template.bash
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

### Step 1 — Configure your HPC server profile

Copy the profile template to your super project:

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

### Step 2 — Build locally (cross-platform for HPC target)

```bash
dna build slurm --apptainer valeria
```

This:
- Builds the slurm Docker image
- Saves it as a `linux/amd64` tar archive to `artifact/apptainer/`
- Generates `artifact/apptainer/build_sif.sh` (run on HPC to convert tar → SIF)

### Step 3 — Generate the Apptainer run script

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

### Step 4 — Transfer to HPC

```bash
rsync -av artifact/apptainer/ user@valeria:/path/to/project/artifact/apptainer/
rsync -av .dockerized_norlab/ user@valeria:/path/to/project/.dockerized_norlab/
```

### Step 5 — Build SIF on HPC server

```bash
# On the HPC server:
bash artifact/apptainer/build_sif.sh
```

### Step 6 — Submit the slurm job

```bash
# On the HPC server — using a job template:
sbatch slurm_job.apptainer.valeria.template.bash

# Or using the generated run script directly:
bash artifact/apptainer/run_apptainer_<sjob-id>.sh
```

## Understanding the Apptainer Pipeline Artifacts

The Apptainer slurm pipeline involves **three related but distinct artifacts**, all sharing
the same HPC profile dotenv configuration:

```
HPC Profile Dotenv (.env.<profile>)
        │
        ├──── sourced by ──── Slurm Job Template (slurm_job.apptainer.<profile>.template.bash)
        │                         │
        │                         └── user edits TODO markers, submits via: sbatch <template>
        │
        └──── sourced by ──── Generated Run Script (run_apptainer_<sjob_id>.sh)
                                  │
                                  └── auto-generated by: dna run slurm <sjob-id> --ga <profile> -- <args>
```

### 1. HPC Profile Dotenv Files (shared config)

**Location (in super project):** `.dockerized_norlab/configuration/hpc_server_profile/.env.<profile>`

Configuration files that define HPC-server-specific environment variables (`DN_PROJECT_USER`,
`DN_PROJECT_PATH`, `APPTAINER_CACHEDIR`, etc.). Used **twice**: locally by DNA at build time
(to bake `DN_PROJECT_USER` into the Docker image) and on the HPC server at runtime (sourced
by both the slurm job templates and generated run scripts).

### 2. Slurm Job Templates (user-editable sbatch scripts)

**Location (in super project):** `slurm_jobs/slurm_job.apptainer.<profile>.template.bash`

Standalone SLURM sbatch scripts copied to the user's project by `dna init`. They include
`#SBATCH` directives, `job_setup_callback()` / `job_teardown_callback()` hooks, and `TODO`
markers for `SJOB_ID` and `python_arguments`. These are the **primary way to submit jobs**
on the HPC server — the user edits the template once and submits via `sbatch`.

### 3. `dna run slurm --ga` Output (auto-generated run scripts)

**Location:** Generated at `artifact/apptainer/run_apptainer_<sjob_id>.sh`

Simpler, auto-generated scripts produced by `dna run slurm <sjob-id> --ga <profile> -- <args>`.
They source the same HPC profile dotenv and use the same `apptainer exec` flags, but have
python args pre-baked from the CLI and no SLURM directives or callbacks. Useful for quick
one-off runs or CI pipelines.

| Aspect | Slurm Job Template | `--ga` Generated Script |
|---|---|---|
| **Purpose** | Full sbatch job with SLURM directives | Minimal run-only script |
| **Customization** | User edits `SJOB_ID`, `python_arguments`, callbacks | Pre-baked from CLI args |
| **SLURM directives** | Yes (`#SBATCH --gres`, `--time`, etc.) | No |
| **Setup/teardown hooks** | Yes | No |
| **How to run** | `sbatch slurm_job.apptainer.<profile>.template.bash` | `bash run_apptainer_<sjob_id>.sh` |
| **Created by** | `dna init` (copied to project) | `dna run slurm --ga` (generated on demand) |

## CLI Reference

### `dna build slurm --apptainer <profile>`

```bash
dna build slurm --apptainer <profile>
```

| Option | Description |
|--------|-------------|
| `--apptainer <profile>` | Build slurm image and save as `linux/amd64` tar archive. Generates `build_sif.sh`. |

Output files in `artifact/apptainer/`:
- `<project>-slurm.<tag>.tar` — Docker tar archive (Apptainer `docker-archive:` compatible)
- `build_sif.sh` — Helper script to run on HPC: `apptainer build <name>.sif docker-archive:<name>.tar`

### `dna save --apptainer <profile> DIRPATH slurm`

```bash
dna save --apptainer <profile> DIRPATH slurm
```

| Option | Description |
|--------|-------------|
| `--apptainer <profile>` | Generate Apptainer artifacts alongside the tar archive |

### `dna run slurm <sjob-id> --generate-apptainer <profile> [OPTIONS] [--] <python-args>`

```bash
dna run slurm <sjob-id> --generate-apptainer <profile> [OPTIONS] [--] <python-args>
dna run slurm <sjob-id> --ga <profile> [OPTIONS] [--] <python-args>
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
| `APPTAINER_TARGET_PLATFORM` | Build time | Docker build platform (default: `linux/amd64`). Sets `DOCKER_DEFAULT_PLATFORM` during `dna build slurm --apptainer` to enforce cross-architecture builds on Apple Silicon Macs. |
| `APPTAINER_ENABLE_GPU` | Runtime | Set to `true` to enable GPU support (`--nv` flag), or `false` for CPU-only jobs. Default: `true`. |
| `APPTAINER_CACHEDIR` | Runtime | Apptainer cache directory on HPC |
| `APPTAINER_TMPDIR` | Runtime | Apptainer temp directory (set to `$SLURM_TMPDIR` in SBATCH script) |

## Slurm Job Templates

| Template | Profile | Description |
|----------|---------|-------------|
| `slurm_job.apptainer.valeria.template.bash` | `valeria` | Valeria HPC standalone job |
| `slurm_job.apptainer.compute_canada.template.bash` | `compute_canada` | Compute Canada standalone job |
| `slurm_job.apptainer.mamba.template.bash` | `mamba` | Mamba HPC standalone job (Apptainer workflow) |
| `slurm_job.apptainer.hpc_hydra.template.bash` | any | Hydra-based standalone job |

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
| `--nv` | _(conditional)_ Enables NVIDIA GPU support. Controlled by `APPTAINER_ENABLE_GPU` in the HPC profile. Set to `false` for CPU-only jobs. |
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
| `image:` | SIF from `apptainer build ... docker-archive:<tar>` | Built from DNA tar archive |
| `volumes:` | `--bind /host:/container[:ro\|:rw]` | Direct mapping |
| `environment:` | `--env-file` (static) + `--env` (dynamic SLURM) | Two-tier strategy |
| `runtime: nvidia` | `--nv` (conditional on `APPTAINER_ENABLE_GPU`) | GPU support |
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
`DN_PROJECT_USER` must be set in the HPC profile env file (e.g., `.env.valeria`, `.env.mamba`) to your
HPC server username **before building**. DNA reads this value at build time and bakes it
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
