# Repository Guidelines

dockerized-norlab-project guidelines and instructions

## Repository Description

_Dockerized-NorLab project application (DNA)_ is the user side companion
of [Dockerized-NorLab (DN)](https://github.com/norlab-ulaval/dockerized-norlab/tree/main) image builder.
It provides a containerized workflow tailor-made for robotic research. Dockerized-NorLab project application (DNA)
manage Dockerized-NorLab (DN) container lifecycle providing functionality for robotic software development, deployment,
testing, continuous integration, slurm job experimentation, and release publishing.
Refer to the repository [README.md](../README.md) for more details.

## Prime directive:

Always comply with guidelines and instructions.

## Repository Guidelines Instructions

- First, review _A2G Framework Guidelines_ specified in `.junie/ai_agent_guidelines/guidelines.a2g_framework.md` for
  additional guidelines.
- Then proceed with the remaining repository guidelines instructions.

## Repository Organization

- `.junie/` contains AI agent related files.
- `.junie/ai_agent_guidelines` contains _AI Agent Guidelines (A2G)_ with entrypoint at
  `.junie/ai_agent_guidelines/README.md`.
- `documentation/` contains the DNA application documentation.
- `src/` contains repository source code.
- `src/bin/dna` is the DNA application entrypoint.
- `src/lib/` contain library files.
- `tests/` contain tests files.
- `tests/tests_bats/` contain N2ST bats framework files that are mainly used for unit-testing.
- `tests/tests_dryrun_and_tests_scripts/` contain integration test (see details below).
- `tests/run_bats_core_test_in_n2st.bash` is a script that will execute all bats unit-tests.
- `tests/run_all_dryrun_and_tests_script.bash` is a script that will execute all dry-run and integration tests.
- `tests/run_all_dryrun_and_tests_script_long_test.bash` is a script that will execute all dry-run and integration tests
  including long tests.
- `utilities/` contain external libraries such as N2ST and NBS.
- `utilities/tmp/dockerized-norlab-project-mock` is use for cloning a fresh copy of a mock "super project"
  from https://github.com/norlab-ulaval/dockerized-norlab-project-mock.git on test execution.
  `dockerized-norlab-project-mock` is a mock of how a user would install and uses DNA. We refer to this as a "super
  project" or the "user side."

## Repository Terminology

- DNA: Is the acronym for _Dockerized-NorLab project application_
- DN: Is the acronym for _Dockerized-NorLab_
