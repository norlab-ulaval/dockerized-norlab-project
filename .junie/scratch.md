
# Scratch

Refactor `dna::check_offline_deploy_service_discovery()` function out of `src/lib/core/utils/load_super_project_config.bash` and put it in a dedicated `offline.bash` utility script in `src/lib/core/utils`.
Update associated tests script accordingly.
Follow guidelines at `.junie/guidelines.md`.
Check if it introduce any breaking change in the code base by runnning both unit-tests and integration tests.
Propose source code change if relevant.
Update or extend corresponding unit-tests and integration tests.
Execute all unit-tests and all integration tests before submiting.



---

New script `test_offline_service_discovery.bash` look loke a integration test.
If it is, it should go in `tests/tests_dryrun_and_tests_scripts` and follow integration test recipe.
Follow guidelines at `.junie/guidelines.md`.
Check if it introduce any breaking change in the code base by runnning both unit-tests and integration tests.
Propose source code change if relevant.
Update or extend corresponding unit-tests and integration tests.
Execute all unit-tests and all integration tests before submiting.


---

Improve `dna` command `up`, `exec`, `attach` and `run` offline deploy service discovery.
Add a check for the presence of a `meta.txt` file.
If so, open it and first, fetch `IMAGE_NAME` env var value and check if that docker image is loaded using docker command e.g., `docker image ls [--filter FILTER|--format STRING]`. 
Next, fetch `SERVICE` env var value and execute `dna COMMAND SERVICE`
Follow guidelines at `.junie/guidelines.md`.
Check if it introduce any breaking change in the code base by runnning both unit-tests and integration tests.
Propose source code change if relevant.
Update or extend corresponding unit-tests and integration tests.
Execute all unit-tests and all integration tests before submiting.

---

Integration test `test_load_directory_persistence.bash` is still failling, review `load.bash` and propose source code change if relevant.
The expected beavior for deploy service is the following: the script should `cd` to `SAVE_DIR_PATH/<SUPER_PROJECT_REPO_NAME>` on exit and the user shell cwd should persist to `SUPER_PROJECT_REPO_NAME` on exit.
Follow guidelines at `.junie/guidelines.md`.
Execute all unit-tests and all integration tests before submiting.


---

I have added logic to `load_super_project_config.bash` for validating that the project root directory has a `.git` directory, implying that it is inded under git version control as required.
Can you review all tests and make sure that it did not introduce braking change.
Check if it introduce any breaking change in the code base by runnning both unit-tests and integration tests.
Propose source code change if relevant. 

---

Why don't you simply execute `source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/load_super_project_config.bash"` before runing the tests instead of copy the function code? 
It woud be less error prone and easier to maintain.
Refactor `test_load.bats` following `test_setup_host_for_running_this_super_project.bats` patern for sourcing dna functions, runnning functions and writing test assertions.

---

Regarding the proposed unit-tests: 
- In `test_load.bats`, dont't mock `dna::cd_to_dna_super_project_root()` as the integration of functions part of the `dna::` namespace are critical.
- Make sure the following test cases are covered:
Case 1: saved directory path not provided and cwd is at the saved directory root;  
Case 2: saved directory path not provided and cwd is deeper in the saved directory structure;
Case 3: saved directory path provided and exist;
Case 4: saved directory path provided and does not exist.
Regarding the proposed integration-tests: 
- Asses if new integration tests would be relevant, implement them if its the case. 
- However, there's already an integration test called `test_save_load_pipeline.bash` that cover the full `dna save` -> `dna load` -> `dna [up|run]` pipeline. Check if `test_save_load_pipeline.bash` would be better suited for covering integration test.
- If you find a new integration test script would still be relevant, either for separation of concern or to simplify test logic, then it should follow `guidelines.md` instructions and similar integration tests recipies.
Update your proposed solutions according to those instruction and as always re-run all unit-tests and integration tests before submiting.

---

Improve `src/lib/commands/load.bash` command loading logic.
Make save directory path optional i.e., `dna load [SAVE_DIR_PATH]`. 
If not given, assume we are in `SAVE_DIR_PATH` and check for `meta.txt` existence in cwd or run `dna::cd_to_dna_super_project_root()`.
For the case where `SAVE_DIR_PATH` positional argument is provided, fix `cd` to `SAVE_DIR_PATH` on exit functionality so that it persist on exit.
Follow guidelines at `.junie/guidelines.md`.
Update `test_load.bats` accordingly.
Create at least one test case per new command argument and/or options, update current tests cases otherwise.
Test relevant option and arguments combinaison.
Check if it introduce any breaking change in the code base by runnning both unit-tests and integration tests.
Propose source code change if relevant.
Execute all unit-tests and all integration tests before submiting.

---

Update a bats test at `tests/tests_bats/test_run.bats` considering change made to `src/lib/commands/run.bash`.
Update a bats test at `tests/tests_bats/test_up.bats` considering change made to `src/lib/commands/up.bash`.
Update a bats test at `tests/tests_bats/test_attach.bats` considering change made to `src/lib/commands/attach.bash`.
Update a bats test at `tests/tests_bats/test_exec.bats` considering change made to `src/lib/commands/exec.bash`.
Follow guidelines at `.junie/guidelines.md`.
Create at least one test case per new command argument and/or options.
Test relevant option and arguments combinaison.
You can mock their corresponding functions as the intended purposes of this test file is for unit-testing the CLI functionalities.

---

Implement bats tests for `install.bash`.
Follow guidelines at `.junie/guidelines.md`.
Inspire yourself with `tests/tests_bats/test_init.bats`.
Create at least one test case per cli options.
Don't mock _helper functions_ and don't mock `dna::install_dockerized_norlab_project_on_host`, those are the functions that we need to test.
Don't mock `n2st::seek_and_modify_string_in_file` function, use the real one.

Start by testing all helper functions: `dna::create_bin_dna_to_entrypoint_symlink`, `dna::update_bashrc_dna_path`, `dna::create_entrypoint_symlink_if_requested` and `dna::add_dna_entrypoint_path_to_bashrc_if_requested`.
Then test integration of helper function in `dna::install_dockerized_norlab_project_on_host`.

---

Refactor `src/lib/commands/build.bash` from a command signature `dna build [OPTIONS|--<SERVICE>]` to a command signature `dna build [OPTIONS] [SERVICE]` with `<SERVICE>` being `ci-tests`, `deploy`, `develop`, `slurm`.
Follow guidelines at `.junie/guidelines.md`.
Update `test_build.bats` accordingly.
Create at least one test case per command argument and/or options.
Test relevant option and arguments combinaison.
Check if it introduce any breaking change in the code base by runnning both unit-tests and integration tests.
Propose source code change if relevant.

---

Review the repository source code and highlight implementation details that would prevent user from running `dna` in offline mode but discard the files and directory in the _ignored list_ from your analysis as they are executed while online. 
Files and directory _ignored list_: `install.bash`, `.env.dockerized-norlab-project`, `src/lib/core/docker/container-tools/project_entrypoints/project-ci-tests/`, `src/lib/core/docker/container-tools/project_entrypoints/project-slurm/`, `src/lib/core/docker/container-tools/dn_project_core.*.bash`, `src/lib/core/docker/docker-compose.project.build.*.yaml`, `src/lib/core/docker/docker-compose.project.run.ci-tests.yaml`, `src/lib/core/docker/docker-compose.project.run.slurm.yaml`, `setup_host_dna_requirements.bash`, `src/lib/core/docker/Dockerfile.*`, `build.*.bash`, `project_validate.*.bash` or any build logic. 
Consider in your analysis that environment variables from `src/lib/core/docker/.env.dna-internal` can be set before sourcing it.
Consider that the DNA repository will be cloned with NBS and N2ST submodule on the remote host before going offline.
Suggest possible refactoring that would mitigate the highlighted issues and/or assess if those are required implementation details.
Follow guidelines at `.junie/guidelines.md`.


---

Refactor `dna::run_command()` function signatures, which curently follow signature `dna run SERVICE [OPTIONS]`, to `dna run [OPTIONS] SERVICE`.
Inspire yourself with `src/lib/commands/up.bash`.
Develop and deploy version should have signature `dna run [OPTIONS] develop|deploy [-- COMMAND [ARGS...]]`.
Ci-tests version should have signature `dna run [OPTIONS] ci-tests`.
Slurm version should have signature `dna run [OPTIONS] slurm <sjob-id>`.
Update corresponding bats tests.
Execute all unit-tests and all integration tests before submiting.
Follow guidelines at `.junie/guidelines.md`.

---
