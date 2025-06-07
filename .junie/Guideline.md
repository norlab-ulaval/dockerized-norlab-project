
 # Dockerized-NorLab-Project guideline

Stand-alone version with a PATH-accessible bash script approach

### General Requirements:
- Path management:
  - Case 1 › system wide:
    - via symlink `/usr/local/bin/dnp` → `/path/to/dockerized-norlab-project/src/bin/dnp`;
    - via `~/.bashrc` ← `PATH=${PATH}:${DNP_PATH}:${NBS_PATH}:${N2ST_PATH}`.
  - Case 2 › manual load: 
    - each super project can use optionally the env var `DNP_PATH`, `NBS_PATH` and `N2ST_PATH` define in `.env.super-project-name`.
- Dont repeat yourself. Use already implemented code such as:
  - `load_repo_main_dotenv.bash`
  - `import_dnp_lib.bash`
  - `load_super_project_config.bash`
  - N2ST library
  - NBS library
  
### Repository development organization
- `src/bin/dnp` is the DNP application entrypoint
- `src/lib/` contain lybrary files
- `tests/` contain tests files
- `tests/tests_bats/` contain N2ST bats framework files that are mainly used for unit-testing
- `tests/tests_dryrun_and_tests_scripts/` contain integration test (see details bellow)
- `utilities` contain external libraries such as N2ST and NBS
- `utilities/tmp/dockerized-norlab-project-mock` is use for cloning a fresh copy of a mock "super project" from https://github.com/norlab-ulaval/dockerized-norlab-project-mock.git on test execution.
  `dockerized-norlab-project-mock` is a mock of how a user would install and uses DNP. We refer to this as a "super project" or the "user side".

### Tests Requirements:
- All tests in the `tests` directory must pass
- Unit-tests and Integration tests:
  - All new script or functionality need to have (either or both):
    - Unit-tests: 
      - use N2ST bats tests tools for unit-test. See `run_bats_core_test_in_n2st.bash` and `tests/tests_bats` directory;
      - a corresponding bats unit-test `.bats` file in the `tests/tests_bats` directory.
    - Integration tests: 
      - those are test case where there is multiple script interacting whith each other or we want to assess execution from begining to end;
      - those tests are devided in two categories: 
        - dryrun: either make use of a `--dry-run` flag implemented in the script or make use of the docker `--dry-run` flag  
        - test: all other integration test case that are not dryrun
      - use NBS tests tools for integration-test. See `run_all_dryrun_and_tests_script.bash`;
      - a corresponding `test_*` or `dryrun_*` script in the `tests/tests_dryrun_and_tests_scripts` directory. 
- You can mock shell core command an docker command but dont mock the function that is actualy tested.
- One test file (`.bats` or `.bash`) per coresponding source code script.
- Identify relevant test cases e.g., behavior validation, error handling, desired user feedback, ...   
- Divide test file by test case: one test function per test case.
- Provide a summary explanation of the test case: 
  - what does it test for; 
  - what's the test expected outcome (i.e, should it pass or fail); 
  - if you do mock something, justify why.
- Use bats framework `bats-file` helper library provide tools for temporary directory management, such as the `temp_make` and `temp_del` functions.
  Refenrece https://github.com/bats-core/bats-file?tab=readme-ov-file#working-with-temporary-directories

### Tests execution
- Don't directly execute `.bats` files, instead execute from the repository root `bash ./tests/run_bats_core_test_in_n2st.bash tests/tests_bats/<bats-file-name>.bats`.
- Dont set tests script in executable mode instead execute them with `bash <the-script-name>.bash`. 
 
