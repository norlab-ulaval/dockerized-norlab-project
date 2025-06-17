
# Bats unit-test generation 

Implement a bats test for `src/lib/commands/run.bash`.
Follow the instructions in `.junie/Guideline.md`.
Inspire yourself with `tests/tests_bats/test_up.bats`.
Create at least one test case per command argument and/or options.
Test relevant option and arguments combinaison.
You can mock their corresponding functions as the intended purposes of this test file is for unit-testing the CLI functionalities.



---

Update a bats test at `tests/tests_bats/test_run.bats` considering change made to `src/lib/commands/run.bash`.
Follow the instructions in `.junie/Guideline.md`.
Create at least one test case per new command argument and/or options.
Test relevant option and arguments combinaison.
You can mock their corresponding functions as the intended purposes of this test file is for unit-testing the CLI functionalities.


---

Implement a bats test for `src/lib/core/utils/setup_host_dnp_requirements.bash`.
Inspire yourself with `tests/tests_bats/test_init.bats`.
Follow the instructions in `.junie/Guideline.md`.
Create at least one test case per steps:
- Install docker requirements;
- CUDA toolkit path.
You can mock docker commands and git commands. 
Don't mock `install_docker_tools.bash` function.

---

Implement bats tests for `install.bash`.
Follow the instructions in `.junie/Guideline.md`.
Inspire yourself with `tests/tests_bats/test_init.bats`.
Create at least one test case per cli options.
Don't mock _helper functions_ and don't mock `dnp::install_dockerized_norlab_project_on_host`, those are the functions that we need to test.
Don't mock `n2st::seek_and_modify_string_in_file` function, use the real one.


Start by testing all helper functions: `dnp::create_bin_dnp_to_entrypoint_symlink`, `dnp::update_bashrc_dnp_path`, `dnp::create_entrypoint_symlink_if_requested` and `dnp::add_dnp_entrypoint_path_to_bashrc_if_requested`.
Then test integration of helper function in `dnp::install_dockerized_norlab_project_on_host`.

---

Refactor `src/lib/commands/build.bash` from a command signature `dnp build [OPTIONS|--<SERVICE>]` to a command signature `dnp build [OPTIONS] [SERVICE]` with `<SERVICE>` being `ci-tests`, `deploy`, `develop`, `slurm`.
Follow the instructions in `.junie/Guideline.md`.
Update `test_build.bats` accordingly.
Create at least one test case per command argument and/or options.
Test relevant option and arguments combinaison.
Check if it introduce any breaking change in the code base by runnning both unit-tests and integration tests.
Propose code base change if relevant. 

---


