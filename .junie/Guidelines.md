
 # Dockerized-NorLab-Project guidelines

Stand-alone version with a PATH-accessible bash script approach

## General Requirements:
- Don't repeat yourself. Use already implemented code such as:
  - `load_repo_main_dotenv.bash`
  - `import_dnp_lib.bash`
  - `load_super_project_config.bash`
  - N2ST library
  - NBS library
- Path management:
  - Case 1 › system wide:
    - via symlink `/usr/local/bin/dnp` → `/path/to/dockerized-norlab-project/src/bin/dnp`;
    - via `~/.bashrc` ← `PATH=${PATH}:${DNP_PATH}:${NBS_PATH}:${N2ST_PATH}`.
  - Case 2 › manual load: 
    - each super project can optionally use the env var `DNP_PATH`, `NBS_PATH` and `N2ST_PATH` define in `.env.dockerized-norlab-project`.
  
## Repository Development Organization
- `src/bin/dnp` is the DNP application entrypoint
- `src/lib/` contain lybrary files
- `tests/` contain tests files
- `tests/tests_bats/` contain N2ST bats framework files that are mainly used for unit-testing
- `tests/tests_dryrun_and_tests_scripts/` contain integration test (see details bellow)
- `utilities` contain external libraries such as N2ST and NBS
- `utilities/tmp/dockerized-norlab-project-mock` is use for cloning a fresh copy of a mock "super project" from https://github.com/norlab-ulaval/dockerized-norlab-project-mock.git on test execution.
  `dockerized-norlab-project-mock` is a mock of how a user would install and uses DNP. We refer to this as a "super project" or the "user side".

## Tests Requirements:

### Testing Strategy
- Inspect the tested script/functions for business logic related error or implementation error. Propose correction before going forward if any. 
- Write tests who chalenge the intended functionality or behavior.
- Write **Unit-tests** and/or **Integration tests**:
  - All new scripts or functionalities need to have (either or both):
    - **Unit-tests**: 
      - use [N2ST](https://github.com/norlab-ulaval/norlab-shell-script-tools) bats tests tools for unit-test (See `tests/run_bats_core_test_in_n2st.bash` script) and a corresponding bats unit-test `.bats` file in the `tests/tests_bats/` directory. Bats tests will be running in a docker container in complete isolation with a copy of the source code.
    - **Integration tests**: 
      - those are test case where there is multiple script interacting whith each other or we want to assess execution from begining to end;
      - those tests are devided in two categories: 
        - Dryrun: either make use of a `--dry-run` flag implemented in the script or make use of the docker `--dry-run` flag;  
        - Test: all other integration test case that are not dryrun.
      - Use [NBS](https://github.com/norlab-ulaval/norlab-build-system) tests tools for integration-test (See `tests/run_all_dryrun_and_tests_script.bash` script) and a corresponding `test_*` or `dryrun_*` script in the `tests/tests_dryrun_and_tests_scripts/` directory. 
- One test file (`.bats` or `.bash`) per coresponding source code script.
- Identify relevant test cases e.g., behavior validation, error handling, desired user feedback, ...   
- If the tested script implement helper functions (i.e., support function meant to be used by the main function), test those functions first.
- Divide test file by test cases: one test function per test case.
- Provide a summary explanation of the test case: 
  - What does it test for; 
  - What's the test expected outcome (i.e, should it pass or fail); 
  - If you do mock something, justify why.
- All tests in the `tests/` directory must pass.
- The definition fo _Done_ mean that tests where executed and all tests passed.

### Instruction On Mocking
- You can mock shell core command an docker command.
- You can mock `docker [OPTIONS|COMMAND]` commands and `git [OPTIONS|COMMAND]` commands.
- Don't mock the functions that are tested in the tested script.
- Avoid mocking n2st functions, at the expection of those in `src/function_library/prompt_utilities.bash`. For eaxample, instead of re-implementaing `n2st::seek_and_modify_string_in_file`, just load the real one and test that the content of the file at `file_path` has been updated? You can find the real one in `src/function_library/general_utilities.bash`.
- Avoid mocking the `read` command. Instead use `echo 'y'` or `echo 'N'` for piping a keyboard input to the function who use the `read` command which in turn expect a single character, example: `run bash -c "echo 'y' | <the-tested-function>"`. Alternatively, use the `yes [n]` shell command which optionaly send [y|Y|yes] n time, example: `run bash -c "yes 2 | <the-tested-function>"`.

### Instruction On Bats Tests
- Use bats framework `bats-file` helper library provide tools for temporary directory management, such as the `temp_make` and `temp_del` functions. 
  Reference https://github.com/bats-core/bats-file?tab=readme-ov-file#working-with-temporary-directories
- Use bats test function `assert_file_executable` and `assert_file_not_executable` to test executable.
- Use bats test function `assert_symlink_to` and `assert_not_symlink_to` to test symlink.
- You can test symlink `/usr/local/bin/dnp` directly without mocking since bats tests are running in a docker container in complete isolation.

Bats helper library documentation:
  - https://github.com/bats-core/bats-assert
  - https://github.com/bats-core/bats-file
  - https://github.com/bats-core/bats-support

### Bats Tests Execution
- Don't directly execute `.bats` files, instead execute from the repository root `bash ./tests/run_bats_core_test_in_n2st.bash tests/tests_bats/<bats-file-name>.bats`.
- Don't set tests script in executable mode instead execute them with `bash <the-script-name>.bash`. 
