# DNA/DN Docker Compose Overrides and Orchestration

This page explains how DNA (Dockerized‑NorLab project application) discovers and applies Docker Compose override files from your super project, and how the orchestration works across modes and platforms.

## TL;DR

- Put your overrides in `.dockerized_norlab/configuration/overrides/` at the root of your project.
- Two override files are considered automatically for every DNA command that runs or inspects compose files:
  1. `docker-compose.global.override.yaml` — applied to all compose files
  2. `<base-compose-file>.override.yaml` — applied only when that compose file is used (same name as base, but with `.override.yaml` suffix)
- Merge order (first to last):
  1. Base compose file from DNA
  2. `docker-compose.global.override.yaml`
  3. `<base-compose-file>.override.yaml`
  The last file wins for conflicts (standard Docker Compose merge rules).
- Preview the final merged configuration with `dna config`.

---

## Where override files live

Your DNA super project (the one you `dna init`’d) contains a configuration directory:

```
<your-repo>/
└── .dockerized_norlab/
    └── configuration/
        └── overrides/
            ├── docker-compose.global.override.yaml           # optional
            ├── docker-compose.run.darwin.override.yaml       # optional
            ├── docker-compose.run.linux-x86.override.yaml    # optional
            ├── docker-compose.run.jetson.override.yaml       # optional
            ├── docker-compose.run.slurm.override.yaml        # optional
            ├── docker-compose.run.ci-tests.override.yaml     # optional
            ├── docker-compose.build.native.override.yaml     # optional
            └── docker-compose.build.multiarch.override.yaml  # optional
```

Only the files that exist are used. You can keep the directory empty if you don’t need overrides.

The list is not exhaustive — any compose file that DNA uses can have a sibling override file with the same name plus the `.override.yaml` suffix.

---

## How DNA discovers overrides

At runtime, DNA builds the Docker Compose command and injects `-f` flags for overrides in this order:

1. The base compose file, selected by DNA for the current command/mode/platform
2. `-f <super-project>/.dockerized_norlab/configuration/overrides/docker-compose.global.override.yaml` (if present)
3. `-f <super-project>/.dockerized_norlab/configuration/overrides/<base-compose-file>.override.yaml` (if present)

This behavior is implemented by the function `dna::generate_super_project_compose_override_files_flags` in `src/lib/core/utils/load_super_project_config.bash`. It is used by the execution utilities and commands such as:

- `src/lib/core/utils/execute_compose.bash`
- `src/lib/core/utils/cuda_tools.bash`
- `src/lib/core/execute/run.slurm.bash`
- `src/lib/commands/config.bash`

Result: your override files are always considered by `dna build`, `dna run` orchestration, SLURM runs, and `dna config` previews.

---

## Which base compose file is used

DNA ships a set of compose files under `src/lib/core/docker/`. The base file that is used depends on the command and mode/platform.

Common base files:

- Run-time (services):
  - `docker-compose.run.darwin.yaml`
  - `docker-compose.run.linux-x86.yaml`
  - `docker-compose.run.jetson.yaml`
  - `docker-compose.run.ci-tests.yaml`
  - `docker-compose.run.slurm.yaml`
  - These use `extends: ... file: docker-compose.run.main.yaml` to share service definitions (`project-develop`, `project-deploy`, `project-ci-tests`, `project-slurm`).

- Build-time (images):
  - `docker-compose.build.native.yaml`
  - `docker-compose.build.multiarch.yaml`

DNA selects the appropriate file automatically based on the command and flags. For example, `dna config dev linux` uses `docker-compose.run.linux-x86.yaml` while `dna config build` uses `docker-compose.build.native.yaml`.

---

## Preview merged configuration

Use `dna config` to see exactly what Docker Compose will get after merges (including your overrides):

```bash
# Development mode for Linux
$ dna config dev linux

# macOS
$ dna config dev darwin

# Jetson
$ dna config dev jetson

# SLURM service
$ dna config slurm

# Build (native or multi-arch)
$ dna config build
$ dna config build-ma
```

You can pass through Docker flags after `--` if needed, e.g. `dna config dev -- --no-interpolate`.

---

## Simple recipes

Below are minimal patterns you can paste into override files. Keep them simple; Compose merges are additive and last-wins.

### 1) Mount an extra host directory in development

File: `.dockerized_norlab/configuration/overrides/docker-compose.run.linux-x86.override.yaml`

```yaml
services:
  project-develop:
    volumes:
      - ${SUPER_PROJECT_ROOT}/my_extra_tools:/opt/tools:ro
```

Repeat similarly for `docker-compose.run.darwin.override.yaml` if you also develop on macOS.

### 2) Enable a different Docker runtime or GPU visibility

File: `.dockerized_norlab/configuration/overrides/docker-compose.run.linux-x86.override.yaml`

```yaml
services:
  project-develop:
    environment:
      NVIDIA_VISIBLE_DEVICES: "all"   # or a concrete device list, or "void" to disable
      NVIDIA_DRIVER_CAPABILITIES: "all"
    runtime: ${DN_DOCKER_RUNTIME:-nvidia}
```

Note: DNA already auto-configures `DN_DOCKER_RUNTIME`, `NVIDIA_VISIBLE_DEVICES`, and `NVIDIA_DRIVER_CAPABILITIES` when appropriate (see GPU section in the README). Override only if you need a special case.

### 3) Add a sidecar service for observability

File: `.dockerized_norlab/configuration/overrides/docker-compose.global.override.yaml`

```yaml
services:
  grafana:
    image: grafana/grafana:latest
    ports:
      - "3000:3000"
      
  project-develop:
    depends_on:
      - grafana
```

Compose will include this service whenever the current base compose use the `project-develop` service.

### 4) Tweak SLURM run container

File: `.dockerized_norlab/configuration/overrides/docker-compose.run.slurm.override.yaml`

```yaml
services:
  project-slurm:
    labels:
      norlab.dockerized-norlab.project.service: "custom slurm override"
    volumes:
      - ${SUPER_PROJECT_ROOT}/slurm_shared:/mnt/slurm_shared:rw
```

### 5) Add ulimits or sysctls for simulation loads

File: `.dockerized_norlab/configuration/overrides/docker-compose.run.linux-x86.override.yaml`

```yaml
services:
  project-develop:
    ulimits:
      nofile: 65536
    sysctls:
      net.core.somaxconn: 1024
```

### 6) Changing entrypoint only on macOS

File: `.dockerized_norlab/configuration/overrides/docker-compose.run.darwin.override.yaml`

```yaml
services:
  project-develop:
    entrypoint: ["/bin/bash", "-lc", "echo 'Hello macOS' && /entrypoints/up_and_attach.bash"]
```

---

## Orchestration flow (what DNA actually does)

- For run/build/SLURM operations, DNA assembles the `docker compose` command with:
  - the selected base compose file (see list above)
  - `-f` flags for your global and per-file overrides (if present)
  - the requested Docker subcommand and args (e.g., `build`, `run`, `up`)
- GPU configuration is auto-detected and exported via `dna::configure_gpu_capabilities` when relevant; these environment variables are visible to the compose runtime.
- Platform-specific run files (`run.darwin`, `run.linux-x86`, `run.jetson`) `extends` the shared service spec in `run.main.yaml` and add suitable volumes, environment, and host integration for each platform.

You can inspect and verify this behavior using `dna config` before running.

---

## Tips and caveats

- Keep overrides small and intentional; avoid duplicating large sections from the base compose.
- If two override files define the same field, the last one wins. Use `<base-compose-file>.override.yaml` when you need to take precedence over the global override.
- When your change should apply everywhere, prefer `docker-compose.global.override.yaml`.
- If you need different behavior per platform, split changes into platform-specific files (e.g., `run.darwin.override.yaml` vs `run.linux-x86.override.yaml`).
- You can always fall back to direct `docker compose` usage with your own `-f` files for advanced cases, but the built‑in override mechanism should cover most needs while keeping DNA commands consistent.

---

## Related commands

- `dna config` — print the fully merged compose configuration for a given mode/platform
- `dna project validate` — validate compose and environment; picks up your overrides
- `dna build` / orchestration — uses overrides under the hood via DNA utilities


## Navigation

- [← Go to Project Initialization & Configuration](project_initialization_and_configuration.md)
- [← Go to Command Reference](dna.md)
- [← Go to Main README](../README.md)
