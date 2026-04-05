### Containerized Apptainer Integration Tests

End-to-end integration tests for DNA's Apptainer/HPC pipeline using real Apptainer
inside a Docker container (Apptainer-in-Docker).

#### Purpose

Since Apptainer is Linux-only and DNA targets macOS hosts, these tests run inside a
Linux Docker container with Apptainer installed. They validate the **full pipeline**:

```
Docker tar archive → dna_tar_to_apptainer_sif_converter.sh → SIF file → apptainer exec (with DNA flags)
```

#### Prerequisites

- Docker Desktop running
- Internet access (first run pulls `ubuntu:22.04` and installs Apptainer)

#### Usage

```bash
# Run all containerized Apptainer tests
bash tests/tests_containerized_apptainer/run_containerized_apptainer_tests.bash

# Options
bash tests/tests_containerized_apptainer/run_containerized_apptainer_tests.bash --no-cache   # Force rebuild
bash tests/tests_containerized_apptainer/run_containerized_apptainer_tests.bash --keep        # Keep artifacts for debugging
bash tests/tests_containerized_apptainer/run_containerized_apptainer_tests.bash --skip-build  # Reuse existing images
```

#### Test Suite

| Test Script | What It Validates |
|---|---|
| `test_pipeline_tar_to_sif.bash` | Full tar → SIF conversion via DNA's `dna_tar_to_apptainer_sif_converter.sh`, SIF label inspection, basic exec |
| `test_apptainer_exec_flags.bash` | DNA flags with real Apptainer: `--cleanenv`, `--env-file`, `--env`, `--no-home`, `--writable-tmpfs`, `--bind`, `--pwd` |
| `test_generated_run_script.bash` | `dna run slurm --ga` output: script generation, structure, execution, env pass-through |
| `test_slurm_job_template.bash` | `slurm_job.apptainer.*.template.bash` for all profiles: structure, adapted execution, env vars |
| `test_entrypoint_detection.bash` | Entrypoint runtime detection (`DNA_RUNTIME=apptainer`), `APPTAINER_CONTAINER` auto-set, arg pass-through |

#### Architecture

```
run_containerized_apptainer_tests.bash    ← Host-side orchestrator (macOS)
  │
  ├── Builds: Dockerfile.apptainer-test-env   ← Ubuntu + Apptainer installed
  ├── Builds: Dockerfile.mock-slurm-image     ← Minimal mock of DNA slurm image
  │            └── mock_entrypoint.bash        ← Simplified dn_entrypoint.init.bash
  ├── Saves mock image as tar
  ├── Prepares mock super project (HPC profiles, directories)
  │
  └── Runs: docker run --privileged dna-apptainer-test-env
              │
              └── container_test_scripts/
                  ├── run_all_container_tests.bash   ← Test runner (inside container)
                  ├── test_pipeline_tar_to_sif.bash
                  ├── test_apptainer_exec_flags.bash
                  ├── test_generated_run_script.bash
                  ├── test_slurm_job_template.bash
                  └── test_entrypoint_detection.bash
```

#### Notes

- The `--privileged` flag is required because Apptainer needs namespace and fuse support.
- The `.test_artifacts/` directory is created at runtime and cleaned up automatically
  (use `--keep` to preserve for debugging).
- The mock slurm image is a minimal simulation of the real DNA slurm Docker image
  (`Dockerfile.run-slurm`) — it has the same directory structure, environment variables,
  and entrypoint path but does not require the full DN base image stack.
