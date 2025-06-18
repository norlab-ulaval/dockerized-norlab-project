
# Bats unit-test generation 

Implement a bats test for `src/lib/commands/run.bash`.
Follow guidelines at `.junie/Guidelines.md`.
Inspire yourself with `tests/tests_bats/test_up.bats`.
Create at least one test case per command argument and/or options.
Test relevant option and arguments combinaison.
You can mock their corresponding functions as the intended purposes of this test file is for unit-testing the CLI functionalities.



---

Update a bats test at `tests/tests_bats/test_build.bats` considering change made to `src/lib/commands/build.bash`.
Follow guidelines at `.junie/Guidelines.md`.
Create at least one test case per new command argument and/or options.
Test relevant option and arguments combinaison.
You can mock their corresponding functions as the intended purposes of this test file is for unit-testing the CLI functionalities.


---

Implement a bats test for `src/lib/core/utils/setup_host_dnp_requirements.bash`.
Inspire yourself with `tests/tests_bats/test_init.bats`.
Follow guidelines at `.junie/Guidelines.md`.
Create at least one test case per steps:
- Install docker requirements;
- CUDA toolkit path.
You can mock docker commands and git commands. 
Don't mock `install_docker_tools.bash` function.

---

Implement bats tests for `install.bash`.
Follow guidelines at `.junie/Guidelines.md`.
Inspire yourself with `tests/tests_bats/test_init.bats`.
Create at least one test case per cli options.
Don't mock _helper functions_ and don't mock `dnp::install_dockerized_norlab_project_on_host`, those are the functions that we need to test.
Don't mock `n2st::seek_and_modify_string_in_file` function, use the real one.


Start by testing all helper functions: `dnp::create_bin_dnp_to_entrypoint_symlink`, `dnp::update_bashrc_dnp_path`, `dnp::create_entrypoint_symlink_if_requested` and `dnp::add_dnp_entrypoint_path_to_bashrc_if_requested`.
Then test integration of helper function in `dnp::install_dockerized_norlab_project_on_host`.

---

Refactor `src/lib/commands/build.bash` from a command signature `dnp build [OPTIONS|--<SERVICE>]` to a command signature `dnp build [OPTIONS] [SERVICE]` with `<SERVICE>` being `ci-tests`, `deploy`, `develop`, `slurm`.
Follow guidelines at `.junie/Guidelines.md`.
Update `test_build.bats` accordingly.
Create at least one test case per command argument and/or options.
Test relevant option and arguments combinaison.
Check if it introduce any breaking change in the code base by runnning both unit-tests and integration tests.
Propose code base change if relevant. 

---


Review the repository source code and highlight implementation details that would prevent user from running `dnp` in offline mode but discard the files and directory in the _ignored list_ from your analysis as they are executed while online. 
Files and directory _ignored list_: `install.bash`, `.env.dockerized-norlab-project`, `src/lib/core/docker/container-tools/project_entrypoints/project-ci-tests/`, `src/lib/core/docker/container-tools/project_entrypoints/project-slurm/`, `src/lib/core/docker/container-tools/dn_project_core.*.bash`, `src/lib/core/docker/docker-compose.project.build.*.yaml`, `src/lib/core/docker/docker-compose.project.run.ci-tests.yaml`, `src/lib/core/docker/docker-compose.project.run.slurm.yaml`, `setup_host_dnp_requirements.bash`, `src/lib/core/docker/Dockerfile.*`, `build.*.bash`, `project_validate.*.bash` or any build logic. 
Consider in your analysis that environment variables from `src/lib/core/docker/.env.dnp-internal` can be set before sourcing it.
Consider that the DNP repository will be cloned with NBS and N2ST submodule on the remote host before going offline.
Suggest possible refactoring that would mitigate the highlighted issues and/or assess if those are required implementation details.
Follow guidelines at `.junie/Guidelines.md`.

---
In `test_load.bats`, instead of mocking `find`, `grep`, `cut`, `cd`, `pwd`, `command` and `basename` command, use the real one and tests the result using bats assert functionalities as instructed in `Guidelines.md`

---

Follow guidelines at `.junie/Guidelines.md`.
Integration tests `dryrun_load.bash`, `dryrun_save_deploy.bash`, `dryrun_save_develop.bash` and `test_save_load_pipeline.bash` and unit-test `test_load.bats` and `test_save.bats` are all failling. Please investigate and make the required changes. 
Thanks

---

Refactor `dnp::attach_command()` function signatures, which is curently using a `--service SERVICE` flag, to a `dnp attach [OPTIONS] [SERVICE]` function signature.
Inspire yourself with `src/lib/commands/build.bash`.
Repeat the same refactoring procedure for `dnp::exec_command()`, `dnp::run_command()` and `dnp::up_command()`.
Keep `develop` as the default service.
Update corresponding bats tests for all refactored functions.
Execute all unit-tests and all integration tests before submiting
Follow guidelines at `.junie/Guidelines.md`.

