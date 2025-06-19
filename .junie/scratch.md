# Recipies

## General

Follow guidelines at `.junie/guidelines.md`.
Execute all unit-tests and all integration tests before submiting.

Add/refactor/improve TODO 
Check if it introduce any breaking change in the code base by runnning both unit-tests and integration tests.
Propose source code change if relevant. 

Inspire yourself with `TODO`.

## Bats unit-test generation 

Implement a bats test for `src/lib/commands/TODO.bash`.
Follow guidelines at `.junie/guidelines.md`.
Inspire yourself with `tests/tests_bats/test_TODO.bats`.
Create at least one test case per command argument and/or options.
Test relevant option and arguments combinaison.
You can mock their corresponding functions as the intended purposes of this test file is for unit-testing the CLI functionalities.
Propose source code change if relevant. 
Execute all unit-tests and all integration tests before submiting.

## Improve/refactor source code

Refactor/improve `src/lib/commands/TODO.bash`.
TODO
Follow guidelines at `.junie/guidelines.md`.
Update `test_TODO.bats` accordingly.
Create at least one test case per new command argument and/or options, update current tests cases otherwise.
Test relevant option and arguments combinaison.
Check if it introduce any breaking change in the code base by runnning both unit-tests and integration tests.
Propose source code change if relevant.
Execute all unit-tests and all integration tests before submiting.

## Modify proposed tests solutions 

You overcomplicated `TODO` new test cases. 
Don't test flag that are not part of the cli definition even if they are mentioned in the doc.
You only need a test case for TODO

---

In `TODO.bats`, instead of mocking `find`, `grep`, `cut`, `cd`, `pwd`, `command` and `basename` command, use the real one and tests the result using bats assert functionalities as instructed in `guidelines.md`

---

Integration tests `TODO` and `TODO` are all failling. 
Please investigate and make the required changes. 
Always follow guidelines at `.junie/guidelines.md`.

---

The following proposed code in `TODO.bash` is overcomplicated 
```shell
TODO
```
Instead, inspire yourself with `TODO.bash` cli implementation:
```shell
TODO
```
Its clearer, explicit and more intuitive.


---------------------------------------------------------------------------------------------------

# Scratch

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
Refactor `test_load.bats` following `test_setup_host_for_running_this_super_project.bats` patern for sourcing dnp functions, runnning functions and writing test assertions.

---

Regarding the proposed unit-tests: 
- In `test_load.bats`, dont't mock `dnp::cd_to_dnp_super_project_root()` as the integration of functions part of the `dnp::` namespace are critical.
- Make sure the following test cases are covered:
Case 1: saved directory path not provided and cwd is at the saved directory root;  
Case 2: saved directory path not provided and cwd is deeper in the saved directory structure;
Case 3: saved directory path provided and exist;
Case 4: saved directory path provided and does not exist.
Regarding the proposed integration-tests: 
- Asses if new integration tests would be relevant, implement them if its the case. 
- However, there's already an integration test called `test_save_load_pipeline.bash` that cover the full `dnp save` -> `dnp load` -> `dnp [up|run]` pipeline. Check if `test_save_load_pipeline.bash` would be better suited for covering integration test.
- If you find a new integration test script would still be relevant, either for separation of concern or to simplify test logic, then it should follow `guidelines.md` instructions and similar integration tests recipies.
Update your proposed solutions according to those instruction and as always re-run all unit-tests and integration tests before submiting.

---

Improve `src/lib/commands/load.bash` command loading logic.
Make save directory path optional i.e., `dnp load [SAVE_DIR_PATH]`. 
If not given, assume we are in `SAVE_DIR_PATH` and check for `meta.txt` existence in cwd or run `dnp::cd_to_dnp_super_project_root()`.
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
Don't mock _helper functions_ and don't mock `dnp::install_dockerized_norlab_project_on_host`, those are the functions that we need to test.
Don't mock `n2st::seek_and_modify_string_in_file` function, use the real one.

Start by testing all helper functions: `dnp::create_bin_dnp_to_entrypoint_symlink`, `dnp::update_bashrc_dnp_path`, `dnp::create_entrypoint_symlink_if_requested` and `dnp::add_dnp_entrypoint_path_to_bashrc_if_requested`.
Then test integration of helper function in `dnp::install_dockerized_norlab_project_on_host`.

---

Refactor `src/lib/commands/build.bash` from a command signature `dnp build [OPTIONS|--<SERVICE>]` to a command signature `dnp build [OPTIONS] [SERVICE]` with `<SERVICE>` being `ci-tests`, `deploy`, `develop`, `slurm`.
Follow guidelines at `.junie/guidelines.md`.
Update `test_build.bats` accordingly.
Create at least one test case per command argument and/or options.
Test relevant option and arguments combinaison.
Check if it introduce any breaking change in the code base by runnning both unit-tests and integration tests.
Propose source code change if relevant.

---

Review the repository source code and highlight implementation details that would prevent user from running `dnp` in offline mode but discard the files and directory in the _ignored list_ from your analysis as they are executed while online. 
Files and directory _ignored list_: `install.bash`, `.env.dockerized-norlab-project`, `src/lib/core/docker/container-tools/project_entrypoints/project-ci-tests/`, `src/lib/core/docker/container-tools/project_entrypoints/project-slurm/`, `src/lib/core/docker/container-tools/dn_project_core.*.bash`, `src/lib/core/docker/docker-compose.project.build.*.yaml`, `src/lib/core/docker/docker-compose.project.run.ci-tests.yaml`, `src/lib/core/docker/docker-compose.project.run.slurm.yaml`, `setup_host_dnp_requirements.bash`, `src/lib/core/docker/Dockerfile.*`, `build.*.bash`, `project_validate.*.bash` or any build logic. 
Consider in your analysis that environment variables from `src/lib/core/docker/.env.dnp-internal` can be set before sourcing it.
Consider that the DNP repository will be cloned with NBS and N2ST submodule on the remote host before going offline.
Suggest possible refactoring that would mitigate the highlighted issues and/or assess if those are required implementation details.
Follow guidelines at `.junie/guidelines.md`.


---

Refactor `dnp::run_command()` function signatures, which curently follow signature `dnp run SERVICE [OPTIONS]`, to `dnp run [OPTIONS] SERVICE`.
Inspire yourself with `src/lib/commands/up.bash`.
Develop and deploy version should have signature `dnp run [OPTIONS] develop|deploy [-- COMMAND [ARGS...]]`.
Ci-tests version should have signature `dnp run [OPTIONS] ci-tests`.
Slurm version should have signature `dnp run [OPTIONS] slurm <sjob-id>`.
Update corresponding bats tests.
Execute all unit-tests and all integration tests before submiting.
Follow guidelines at `.junie/guidelines.md`.

---
