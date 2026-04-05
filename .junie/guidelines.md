# Repository Guidelines

dockerized-norlab-project guidelines and instructions


## Repository Description

_Dockerized-NorLab project application (DNA)_ is the user side companion
of [Dockerized-NorLab (DN)](https://github.com/norlab-ulaval/dockerized-norlab/tree/main) image builder.
It provides a containerized workflow tailor-made for robotic research. Dockerized-NorLab project application (DNA)
manage Dockerized-NorLab (DN) container lifecycle providing functionality for robotic software development, deployment,
testing, continuous integration, slurm job experimentation, and release publishing.
Refer to the repository [README.md](../README.md) for more details.


## Repository Guidelines Instructions

1. First, review and learn _A2G Framework Guidelines_ specified in
   `.junie/ai_agent_guidelines/guidelines.a2g_framework.md`.
2. Then review the remaining repository guidelines below.
3. **AI agents must follow the mandatory compliance requirements specified below.**


## Prime directive

Always comply with _A2G Framework Guidelines_, _Repository Guidelines_ and _AI operator_ instructions.


## AI Agent Compliance Requirements

All AI agents must:

1. **Always** review A2G guidelines before starting any task
2. **Always** follow A2G file placement decision tree
3. **Always** check workflow mode in `.junie/a2g_config.yml`
4. **Always** apply A2G task verb interpretation protocols

See A2G general guidelines for complete procedures and requirements.


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


## Repository Specific Additional Guidelines

### For Developers: Creating a New Patch

When you introduce changes to DNA that require updates to the super project configuration (e.g., a new required directory or a new template file), you should:

1. **Increment the Version**: Increment`DNA_RELEASE_CONFIG_SCHEME_VERSION` in `.env.dockerized-norlab-project`.
2. **Create a Patch Script**: Create a new script named `config_scheme_<FROM>to<TO>.bash` in `src/lib/core/patches/`.
3. **Use the Template**: Base your script on `src/lib/core/patches/config_scheme_patch_template.bash`.
4. **Define the Logic**: Use the provided helper functions from `patch_helper.bash` to add missing resources:
    - `dna::patch_add_file_if_missing <template_source> <target_dest> <description>`: Adds a file from DNA templates to the super project.
    - `dna::patch_add_directory_if_missing <template_source> <target_dest> <description>`: Adds a directory from DNA templates to the super project.
    - `dna::patch_add_content_if_missing <target_file> <search_string> <content_to_add> <description>`: Appends content to a file if it doesn't already contain the search string.
    - `dna::patch_modify_content <target_file> <search_pattern> <replace_pattern> <description>`: Modifies file content using a search and replace pattern (powered by `sed`).

The `template_source` path is relative to `src/lib/template/`.
The `target_dest` and `target_file` paths are relative to the super project root.
Refer to `documentation/project_initialization_and_configuration.md` for more details.

Proceed with _AI operator_ instructions
