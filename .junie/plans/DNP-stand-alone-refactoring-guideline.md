# Dockerized-NorLab-Project stand-alone refactoring guidelines

Stand-alone version with a PATH-accessible bash script approach
This approach maintains the current bash script structure but makes it accessible from anywhere via the system `PATH`.

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
  
### Tests Requirements:
- All tests in the `tests` directory must pass
- Unit-tests and Integration tests:
  - All new script or functionality need to have (either or both):
    - Unit-tests: 
      - use N2ST bats tests tools for unit-test. See `run_bats_core_test_in_n2st.bash` and `tests/tests_bats` directory;
      - a corresponding bats unit-test `.bats` file in the `tests/tests_bats` directory; 
      - dont run `.bats` file directly, use `tests/run_bats_core_test_in_n2st.bash tests/tests_bats/<bats-file-name>.bats` instead.
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


### Implementation Plan

#### Step 1: Refactor To a Stand-alone Package Structure

Implement the following repository directory structure: 
- create the required new directory
- move the existing directory or file to their new proposed location
- rename files if necesseray 
- DNP main dotenv file `.env.dockerized-norlab-project`
  - `DNP_PATH`, `NBS_PATH` and `N2ST_PATH` as a fallback for handling local install case
  - `DNP_CONFIG_SCHEME_VERSION` env variable set in each dnp project to validate compatibility

```
dockerized-norlab-project/ # (stand-alone version)
├── src/
│   ├── bin/
│   │   ├── dnp # symlink to /usr/local/bin/dnp
│   │   └── dnp-completion.bash <- ice boxed for now 
│   └── lib/ 
│       ├── commands/
│       │   ├── version.bash
│       │   ├── init.bash # project initialization
│       │   ├── build.bash
│       │   ├── up.bash
│       │   ├── down.bash
│       │   ├── run.bash
│       │   ├── validate.bash
│       │   └── ... (other command scripts)
│       ├── core/
│       │   ├── docker/
│       │   │   ├── .env.dnp-internal
│       │   │   ├── Dockerfile.ci-tests.multiarch
│       │   │   ├── Dockerfile.ci-tests.native
│       │   │   ├── Dockerfile.run-slurm
│       │   │   ├── container-tools
│       │   │   │   ├── dn_project_core.build.aarch_aware_build_ros.bash
│       │   │   │   ├── dn_project_core.build.patch.bash
│       │   │   │   ├── dn_project_core.setup.bash
│       │   │   │   └── project_entrypoints
│       │   │   ├── docker-compose.project.build.multiarch.yaml
│       │   │   ├── docker-compose.project.build.native.yaml
│       │   │   ├── docker-compose.project.run.darwin.yaml
│       │   │   ├── docker-compose.project.run.jetson.yaml
│       │   │   ├── docker-compose.project.run.linux-x86.yaml
│       │   │   └── docker-compose.project.run.slurm.yaml
│       │   ├── execute/
│       │   │   ├── build.all.bash
│       │   │   ├── build.all.multiarch.bash
│       │   │   ├── build.ci_tests.bash
│       │   │   ├── build.ci_tests.multiarch.bash
│       │   │   ├── build.deploy.bash
│       │   │   ├── build.develop.bash
│       │   │   ├── cadence/
│       │   │   │   ├── project_run_slurm_docker_cmd.bash
│       │   │   │   └── run_cadence.ci_test.bash
│       │   │   ├── down.bash
│       │   │   ├── validate.all.bash # <-- renamed from dryrun_and_config_test.all.bash
│       │   │   ├── validate.slurm.bash # <-- renamed from dryrun_and_config_test.slurm.bash
│       │   │   ├── run.ci_tests.bash
│       │   │   ├── run.slurm.bash
│       │   │   ├── down.slurm.bash # <-- renamed from run_kill.slurm.bash
│       │   │   ├── up_and_attach.bash
│       │   │   └── ... (other execute scripts)
│       │   └── utils/
│       │       ├── execute_compose.bash
│       │       ├── import_dnp_lib.bash
│       │       ├── load_super_project_config.bash
│       │       ├── setup_host_for_running_this_super_project.bash
│       │       └── validate_super_project_dnp_setup.bash
│       └── template/
│           ├── .dockerized_norlab_project/
│           │   ├── .env.dockerized-norlab-project-mock
│           │   ├── README.md
│           │   ├── configuration/
│           │   │   ├── .env
│           │   │   ├── .env.dnp
│           │   │   ├── .env.local
│           │   │   ├── Dockerfile
│           │   │   ├── README.md
│           │   │   ├── project_entrypoints/
│           │   │   └── project_requirements/
│           │   ├── dn_container_env_variable/
│           │   └── slurm_jobs/
│           ├── artifact/
│           ├── external_data/
│           ├── src/
│           │   ├── README.md
│           │   ├── launcher/
│           │   └── tools/
│           ├── tests/ 
│           ├── .dockerignore
│           ├── .gitignore
│           └── README.md  
├── tests/
├── visual/
├── utilities/
│   ├── dockerized-norlab-project-mock/ <- git worktree https://github.com/norlab-ulaval/dockerized-norlab-project-mock.git
│   │   ├── .dockerized_norlab_project/
│   │   │   ├── .env.dockerized-norlab-project-mock
│   │   │   ├── README.md
│   │   │   ├── configuration/
│   │   │   │   ├── .env
│   │   │   │   ├── .env.dnp
│   │   │   │   ├── .env.local # Referenced by ignore files
│   │   │   │   ├── Dockerfile
│   │   │   │   ├── README.md
│   │   │   │   ├── project_entrypoints/
│   │   │   │   └── project_requirements/
│   │   │   ├── dn_container_env_variable/
│   │   │   └── slurm_jobs/
│   │   ├── artifact/
│   │   ├── external_data/
│   │   ├── src/
│   │   │   ├── README.md
│   │   │   ├── launcher/
│   │   │   └── tools/
│   │   ├── tests/
│   │   ├── .dockerignore
│   │   ├── .gitignore
│   │   └── README.md
│   ├── norlab-build-system/ <- git submodule https://github.com/norlab-ulaval/norlab-build-system.git
│   └── norlab-shell-script-tools/ <- git submodule https://github.com/norlab-ulaval/norlab-shell-script-tools.git
├── install.bash
├── load_repo_main_dotenv.bash
├── README.md
├── .dockerignore
├── .gitignore
└── .env.dockerized-norlab-project ← declare DNP_ROOT, DNP_MOCK_SUPER_PROJECT_ROOT, N2ST and NBS path 
```


#### Step 2: Create a Main Entry Point Script

##### Requirements:
- a main entrypoint: `dnp` with command (`build`, `up`, `down`, `run`, `init`, `validate`, `config`, `version`, `super`, `update`) linked to corresponding bash script
- `dnp` need to executable 
- `dnp` need to be in `PATH`
- a project dnp config discovery mechanism so that we can do `cd path/to/repo/ && dnp build` instead of `cd path/to/repo/ && dnp build path/to/repo/path/to/config`

#### Step 3: Create a init command Script

The script will initialize the DNP user side resources.
  1. validate that the user executed the command `dnp init` from the super project repository root, return an explicative error message otherwise
  2. create the `.dockerized_norlab_project` directory in the user super project root
  3. copy the configuration template files
  4. create the `.env.user-super-project` file using N2ST script
  5. initialize any placeholder environment variable if needed

##### Requirements:
- Command `dnp init` to initialize a project:
  - create `.env.<super-project-name>` dotenv file with environment variable `DNP_CONFIG_SCHEME_VERSION` env variable set in each dnp project to validate compatibility
  - create the super project config directory and files:
    - `.dockerized_norlab_project` and all subdirectory
    - all super project repository required directory as tested by `validate_super_project_dnp_setup.bash`
  - need to pass `validate_super_project_dnp_setup.bash` tests

```
user-super-project/
├── .dockerized_norlab_project/ # DNP project user side specific configuration
│   ├── configuration/
│   │   ├── .env
│   │   ├── .env.dnp
│   │   ├── .env.local # Referenced by ignore files
│   │   ├── Dockerfile
│   │   ├── README.md
│   │   ├── project_entrypoints/
│   │   └── project_requirements/
│   ├── dn_container_env_variable/ # Referenced by ignore files
│   │   └── .env.dn_expose_user_super_project # auto generated
│   ├── slurm_jobs/
│   ├── visual/
│   ├── README.md
│   └── .env.dockerized-norlab-project-mock
├── artifact/
├── external_data/
├── src/
│   ├── README.md
│   ├── launcher/
│   └── tools/
├── tests/
...
├── .dockerignore
├── .gitignore
└── README.md
```


#### Step 4: Implement `build`, `up`, `down`, `run` and `validate` Command Scripts

##### Requirements:
- Each command script would implement highlevel user logic
- Should be intuitive to use
- Commands:
  - - Command `dnp [build|up|down|run]` act as an abstraction layer that hide the complexity of the `core/execute/` scripts by selecting the proper specialize script among that directory base on user input, e.g., 
    - command `dnp build` would execute the `core/execute/build.all.bash` script;
    - command `dnp build --multiach` would execute the `core/execute/build.all.multiarch.bash` script.
  - Command `dnp config <argument>` to tests compose config with interpolated value and to show it in console
    - one of those argument `[dev [darwin|linux|jetson]|deploy|ci-tests|slurm|release]` to select which compose file and service to to test and show
    - under the hood `dnp config` would execute `execute import_dnp_lib.bash` and `load_super_project_config.bash` and then `docker-compose --file docker-compose.project.*.*.yaml config` command
  - Command `dnp validate` to execute dryrun and config tests
  - Command `dnp super validate` to test super project directory setup by executing `validate_super_project_dnp_setup.bash`
  - Command `dnp super show` to print consolidated and interpolated dotenv config files to console
  - Command `dnp config` to tests compose config with interpolated value and to show it in console

#### Step 5: Implement `version` Command Scripts
This script would simply read and print to console the current local repository version from the `version.txt` file created and updated by semantic-release github action.


#### Step 6:  DNP installation Script

Create an installation script that will will steup the user host computer for using `dockerized-norlab-project` as a stand-alone application.

##### Requirements:
- implement DNP path resolution install options:
  - Option 1: system wide (default):  
    - add a symlink from the `/path/to/cloned/dockerized-norlab-project/src/bin/dnp` to `/user/local/bin/dnp`
    - make `dockerized-norlab-project/src/bin/dnp` excutable
  - Option 2: flag `--skip-system-wide-symlink-install` to skip option 1
  - Option 3: flag `--add-dnp-path-to-bashrc` to add DNP cloned repository path (`DNP_PATH`) to `~/.bashrc`
- implement an `--help` flag with proper documentation
- a `--yes` flag to bypass user interactive installation
- execute `setup_host_for_running_this_super_project.bash` to setup *dockerized-norlab-project* requirement on this host computer
- execute `validate_super_project_dnp_setup.bash` to validate install at the end


#### (iceboxed for now) Step 7: Command `dnp update` 
To check DNP config scheme version, update DNP cloned repo on host and if required execute a per-version update script to modify project DNP config 


### Usage Example

After installation, users would interact with the system like this:

```bash
# Initialize a new project
cd ~/pat/to/my/super-project
dnp init my-project

# Build Docker images
dnp build --multiarch

# Start and attach to a container
dnp up --service develop

# Stop containers
dnp down
```
