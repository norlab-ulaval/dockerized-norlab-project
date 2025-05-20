#!/usr/bin/env bats
#
# Usage in docker container
#   $ REPO_ROOT=$(pwd) && RUN_TESTS_IN_DIR='tests'
#   $ docker run -it --rm -v "$REPO_ROOT:/code" bats/bats:latest "$RUN_TESTS_IN_DIR"
#
#   Note: "/code" is the working directory in the bats official image
#
# bats-core ref:
#   - https://bats-core.readthedocs.io/en/stable/tutorial.html
#   - https://bats-core.readthedocs.io/en/stable/writing-tests.html
#   - https://opensource.com/article/19/2/testing-bash-bats
#       ↳ https://github.com/dmlond/how_to_bats/blob/master/test/build.bats
#
# Helper library: 
#   - https://github.com/bats-core/bats-assert
#   - https://github.com/bats-core/bats-support
#   - https://github.com/bats-core/bats-file
#

BATS_HELPER_PATH=/usr/lib/bats
if [[ -d ${BATS_HELPER_PATH} ]]; then
  load "${BATS_HELPER_PATH}/bats-support/load"
  load "${BATS_HELPER_PATH}/bats-assert/load"
  load "${BATS_HELPER_PATH}/bats-file/load"
  load "${SRC_CODE_PATH}/${N2ST_BATS_TESTING_TOOLS_RELATIVE_PATH}/bats_helper_functions"
  load "${SRC_CODE_PATH}/tests/tests_bats/bats_testing_tools/bats_helper_functions_local"
  load "${SRC_CODE_PATH}/tests/tests_bats/bats_testing_tools/norlab_project_template_helper_functions.bash"
  #load "${BATS_HELPER_PATH}/bats-detik/load" # << Kubernetes support
else
  echo -e "\n[\033[1;31mERROR\033[0m] $0 path to bats-core helper library unreachable at \"${BATS_HELPER_PATH}\"!" 1>&2
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ====Setup========================================================================================

TESTED_FILE="initialize_norlab_project_template.bash"
#TESTED_FILE_PATH="."

# executed once before starting the first test (valide for all test in that file)
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  TNP_PATH_PARENT=${BATS_DOCKER_WORKDIR}

  cd "${TNP_PATH_PARENT:?err}" || exit 1
  set -o allexport
  source tests/.env.tnp_test_values || exit 1
  set +o allexport

  # Setup git for testing commit logic
  git config --global user.email "bats_tester@example.com"
  git config --global user.name "bats_tester"

  ## Uncomment the following for debug, the ">&3" is for printing bats msg to stdin
#  pwd >&3 && tree -L 1 -a -hug >&3

}

# executed before each test
setup() {

  cd "/code/template-norlab-project" || exit 1

  # Note:
  #   - "temp_del" is a bats-file lib function which make a tmp directory guarantee unique
  #   - ref https://github.com/bats-core/bats-file#working-with-temporary-directories
  TEST_TEMP_DIR="$(temp_make)"
  BATS_DOCKER_WORKDIR="${TEST_TEMP_DIR}/template-norlab-project"

  # Note: quick hack to solve the TeamCity server "error: object directory ... does not exist;
  #       check .git/objects/info/alternates". The idea is to clone when the script is run on a TC
  #       server as we know the code was committed to a branch and copy when when locally as the
  #       code might not be committed yet.
  if [[ ${TEAMCITY_VERSION} ]]; then
    echo -e "     [\033[1mN2ST bats container\033[0m] Case TC run" # >&3

    cd "${TEST_TEMP_DIR}" || exit 1
    echo -e "     [\033[1mN2ST bats container\033[0m] Git clone ${TNP_GIT_REMOTE_URL:?err}" # >&3

    git clone  --branch "${TNP_TEAMCITY_PR_SOURCE}" "$TNP_GIT_REMOTE_URL"

    cd "${TNP_GIT_NAME}"
    echo -e "     [\033[1mN2ST bats container\033[0m] cwd=$(pwd)" # >&3

    git fetch --all
    git submodule update --remote --recursive --init

    echo -e "     [\033[1mN2ST bats container\033[0m] Git checkout ${TNP_TEAMCITY_PR_SOURCE}" # >&3
    git checkout --recurse-submodules "${TNP_TEAMCITY_PR_SOURCE}"

  else
    echo -e "     [\033[1mN2ST bats container\033[0m] Copy \"template-norlab-project/\" to ${TEST_TEMP_DIR}"

    # Clone "template-norlab-project/" directory content in tmp directory
    # -p for preserve time and mode
    cp -R -p "/code/template-norlab-project/" "${TEST_TEMP_DIR}/"
    cd "${BATS_DOCKER_WORKDIR}" || exit 1

  fi

#  echo -e "\n› Pre test directory state" >&3 && pwd >&3 && tree -L 1 -a -hug >&3
}

# ====Teardown=====================================================================================

# executed after each test
teardown() {
#  bats_print_run_env_variable_on_error
#  echo -e "\n› Post test directory state" >&3 && pwd >&3 && tree -L 1 -a -hug >&3

  # Reset "$TEST_TEMP_DIR/template-norlab-project/" directory
  # Note:
  #   - "temp_del" is a bats-file lib function
  #   - ref https://github.com/bats-core/bats-file#working-with-temporary-directories
  temp_del "$TEST_TEMP_DIR"
}

## executed once after finishing the last test (valide for all test in that file)
#teardown_file() {
#
#}

# ====Test cases===================================================================================
@test "test directory backup › ok" {
#  skip "tmp dev" # ToDo: on task end >> delete this line ←

  assert_dir_exist "/code/template-norlab-project"
  assert_dir_exist "${TEST_TEMP_DIR}/template-norlab-project"

  assert_equal $(pwd) "${TEST_TEMP_DIR}/template-norlab-project"

#  assert_dir_not_exist /code/template-norlab-project/template-norlab-project-backup

}

@test "execute from wrong directory › expect fail" {
#  skip "tmp dev" # ToDo: on task end >> delete this line ←
#  # Note:
#  #  - "echo 'Y'" is for sending an keyboard input to the 'read' command which expect a single character
#  #    run bash -c "echo 'Y' | source ./function_library/$TESTED_FILE"
#  #  - Alt: Use the 'yes [n]' command which optionaly send n time


  cd .. || exit 1
  assert_file_exist template-norlab-project/README.norlab_template.md

  run bash ./template-norlab-project/$TESTED_FILE

  assert_failure 1
  assert_output --regexp .*"\[ERROR\]".*"'initialize_norlab_project_template.bash' script should be executed from the project root".*

}

@test "source file › expect fail" {
#  skip "tmp dev" # ToDo: on task end >> delete this line ←
  #  - Alt: Use the 'yes [n]' command which optionaly send n time

  run bash -c "yes 1 | source ./$TESTED_FILE"
  assert_failure 1
  assert_output --regexp .*"\[ERROR\]".*"This script must be run with bash i.e.".*"bash initialize_norlab_project_template.bash"
}

@test "Default case › NBS N2ST Semantic-Release and NorLab readme  › expect pass" {
##  skip "tmp dev" # ToDo: on task end >> delete this line ←

  # Note: \n is to simulate the return key
  # Install NBS › Y
  # Install N2ST › Y
  # Semantic-Release › Y
  # Project env var prefix › TMP1
  # Install NorLab readme › Y
  local TEST_CASE="yyyTMP1\ny"

  norlab_project_template_directory_reset_check

  # ....Execute initialize_norlab_project_template.bash............................................
  run bash -c "echo -e \"${TEST_CASE}\" | bash ./$TESTED_FILE"
  assert_success

  # ....Check submodule cloning....................................................................
  cd "${BATS_DOCKER_WORKDIR}" || exit 1
  assert_dir_exist .git
  assert_file_exist .gitmodules

  # ....Check NBS install..........................................................................
  check_NBS_is_installed

  # ....Check N2ST install.........................................................................
  check_N2ST_is_installed

  # ....Check Semantic-Release install.............................................................
  check_semantic_release_is_installed

  # ....Set main readme file to NorLab.............................................................
  assert_file_exist NORLAB_PROJECT_TEMPLATE_INSTRUCTIONS.md
  assert_file_exist README.md
  assert_file_not_exist README.norlab_template.md
  assert_file_not_exist README.vaul_template.md

  assert_file_contains README.md "src=\"/visual/norlab_logo_acronym_dark.png"

  # ....Check teardown.............................................................................
  check_norlab_project_template_teardown

  # .....env file manual assessment ...............................................................
#  more .env.template-norlab-project  >&3
  set -o allexport
  source .env.template-norlab-project
  set +o allexport

}

@test "Validate git add steps and gitignore configuration › expect pass" {
##  skip "tmp dev" # ToDo: on task end >> delete this line ←

  # Note: \n is to simulate the return key
  # Install NBS › Y
  # Install N2ST › Y
  # Semantic-Release › Y
  # Project env var prefix › TMP1
  # Install NorLab readme › Y
  local TEST_CASE="yyyTMP2\ny"

  norlab_project_template_directory_reset_check

  # ....Execute initialize_norlab_project_template.bash............................................
  run bash -c "echo -e \"${TEST_CASE}\" | bash ./$TESTED_FILE"
  assert_success

  # ....Check submodule cloning....................................................................
  cd "${BATS_DOCKER_WORKDIR}" || exit 1
  assert_dir_exist .git
  assert_file_exist .gitmodules

  # ....Check git commit feedback..................................................................
  refute_output --regexp "Commit".*"create mode".*".idea/".*
}

@test "Prefix substitution and changelog reset › expect pass" {
#  skip "tmp dev" # ToDo: on task end >> delete this line ←

  # Note: \n is to simulate the return key
  # Install NBS › Y
  # Install N2ST › Y
  # Semantic-Release › Y
  # Project env var prefix › my_project
  # Install NorLab readme › Y
  local TEST_CASE="yyymy_project\ny"

  norlab_project_template_directory_reset_check

  # ....Execute initialize_norlab_project_template.bash............................................
  run bash -c "echo -e \"${TEST_CASE}\" | bash ./$TESTED_FILE"
  assert_success

  # ....Modify .env project environment variable prefix............................................
  cd "${BATS_DOCKER_WORKDIR}" || exit 1
  assert_file_contains .env.template-norlab-project "^MY_PROJECT_PROMPT_NAME.*"
  assert_file_contains .env.template-norlab-project "^MY_PROJECT_GIT_REMOTE_URL.*"
  assert_file_contains .env.template-norlab-project "^MY_PROJECT_GIT_NAME.*"
  assert_file_contains .env.template-norlab-project "^MY_PROJECT_PATH.*"
  assert_file_contains .env.template-norlab-project "^MY_PROJECT_SRC_NAME.*"
  assert_file_contains .env.template-norlab-project "^PROJECT_PROMPT_NAME='MY_PROJECT'"

  # ....Modify bats test related files.............................................................
  assert_file_not_contains tests/tests_bats/test_template.bats "^@test \"n2st::.*"
  assert_file_contains tests/tests_bats/test_template.bats "^@test \"my_project::.*"

  assert_file_not_contains src/dummy.bash "^function n2st::.*"
  assert_file_contains src/dummy.bash "^function my_project::.*"

  assert_file_not_contains tests/run_bats_core_test_in_n2st.bash "Execute 'template-norlab-project.template' repo"
  assert_file_contains tests/run_bats_core_test_in_n2st.bash "Execute 'template-norlab-project' repo"
  assert_file_not_contains tests/run_bats_core_test_in_n2st.bash "source .env.template-norlab-project.template"
  assert_file_contains tests/run_bats_core_test_in_n2st.bash "source .env.template-norlab-project"

  assert_file_not_contains tests/run_bats_core_test_in_n2st.bash "TNP_"
  assert_file_contains tests/run_bats_core_test_in_n2st.bash "MY_PROJECT_"

  # ....Modify run config related files............................................................
  assert_file_not_contains .run/openATerminalInUbuntuContainer.run.xml "folderName=\"\[TNP\]"
  assert_file_contains .run/openATerminalInUbuntuContainer.run.xml "folderName=\"\[MY_PROJECT\]"

  assert_file_not_contains .run/runBatsTestsAll.run.xml "folderName=\"\[TNP\]"
  assert_file_contains .run/runBatsTestsAll.run.xml "folderName=\"\[MY_PROJECT\]"

  assert_file_not_contains .run/runBatsTestsAll.run.xml "tests/run_bats_core_test_in_n2st.tnp.bash"
  assert_file_contains .run/runBatsTestsAll.run.xml "tests/run_bats_core_test_in_n2st.bash"

  # ....Check Semantic-Release install.............................................................
  check_semantic_release_is_installed

  # ....Check teardown.............................................................................
  check_norlab_project_template_teardown

}

@test "Case no submodule › expect pass" {
#  skip "tmp dev" # ToDo: on task end >> delete this line ←

  # Note: \n is to simulate the return key
  # Install NBS › N
  # Install N2ST › N
  # Semantic-Release › Y
  # Project env var prefix › no_sub
  # Install NorLab readme › Y
  local TEST_CASE="nnyno_sub\ny"

  norlab_project_template_directory_reset_check

  # ....Execute initialize_norlab_project_template.bash............................................
  run bash -c "echo -e \"${TEST_CASE}\" | bash ./$TESTED_FILE"
  assert_success

  # ....Check submodule cloning....................................................................
  cd "${BATS_DOCKER_WORKDIR}" || exit 1
  assert_dir_exist .git
  assert_file_exist .gitmodules
  assert_file_empty .gitmodules

  # ....Check NBS install..........................................................................
  check_NBS_not_installed

  # ....Check N2ST install.........................................................................
  check_N2ST_not_installed

  # ....Check teardown.............................................................................
  check_norlab_project_template_teardown

}

@test "Case install NBS but skip N2ST › expect pass" {
#  skip "tmp dev" # ToDo: on task end >> delete this line ←

  # Note: \n is to simulate the return key
  # Install NBS › Y
  # Install N2ST › N
  # Semantic-Release › Y
  # Project env var prefix › NBS
  # Install NorLab readme › Y
  local TEST_CASE="ynyNBS\ny"

  norlab_project_template_directory_reset_check

  # ....Execute initialize_norlab_project_template.bash............................................
  run bash -c "echo -e \"${TEST_CASE}\" | bash ./$TESTED_FILE"
  assert_success

  # ....Check submodule cloning....................................................................
  cd "${BATS_DOCKER_WORKDIR}" || exit 1
  assert_dir_exist .git
  assert_file_exist .gitmodules

  # ....Check NBS install..........................................................................
  check_NBS_is_installed

  # ....Check N2ST install.........................................................................
  check_N2ST_not_installed

  # ....Check teardown.............................................................................
  check_norlab_project_template_teardown

}

@test "Case install N2ST but skip NBS › expect pass" {
#  skip "tmp dev" # ToDo: on task end >> delete this line ←

  # Note: \n is to simulate the return key
  # Install NBS › N
  # Install N2ST › Y
  # Semantic-Release › Y
  # Project env var prefix ›N2ST
  # Install NorLab readme › Y
  local TEST_CASE="nyyN2ST\ny"

  norlab_project_template_directory_reset_check

  # ....Execute initialize_norlab_project_template.bash............................................
  run bash -c "echo -e \"${TEST_CASE}\" | bash ./$TESTED_FILE"
  assert_success

  # ....Check submodule cloning....................................................................
  cd "${BATS_DOCKER_WORKDIR}" || exit 1
  assert_dir_exist .git
  assert_file_exist .gitmodules

  # ....Check NBS install..........................................................................
  check_NBS_not_installed

  # ....Check N2ST install.........................................................................
  check_N2ST_is_installed

  # ....Check teardown.............................................................................
  check_norlab_project_template_teardown

}

@test "Case skip semantic-release › expect pass" {
#  skip "tmp dev" # ToDo: on task end >> delete this line ←

  # Note: \n is to simulate the return key
  # Install NBS › Y
  # Install N2ST › Y
  # Semantic-Release › N
  # Project env var prefix › NOSEMANTIC
  # Install NorLab readme › Y
  local TEST_CASE="yynNOSEMANTIC\ny"

  norlab_project_template_directory_reset_check

  # ....Execute initialize_norlab_project_template.bash............................................
  run bash -c "echo -e \"${TEST_CASE}\" | bash ./$TESTED_FILE"
  assert_success

  # ....Check submodule cloning....................................................................
  cd "${BATS_DOCKER_WORKDIR}" || exit 1
  assert_dir_exist .git
  assert_file_exist .gitmodules

  # ....Check NBS install..........................................................................
  check_NBS_is_installed

  # ....Check N2ST install.........................................................................
  check_N2ST_is_installed

  # ....Check Semantic-Release install.............................................................
  check_semantic_release_not_installed
}

@test "Case install NorLab readme  › expect pass" {
#  skip "tmp dev" # ToDo: on task end >> delete this line ←

  # Note: \n is to simulate the return key
  # Install NBS › Y
  # Install N2ST › Y
  # Semantic-Release › Y
  # Project env var prefix › TMP1
  # Install NorLab readme › Y
  local TEST_CASE="YYYTMP1\nY"

  norlab_project_template_directory_reset_check

  # ....Execute initialize_norlab_project_template.bash............................................
  run bash -c "echo -e \"${TEST_CASE}\" | bash ./$TESTED_FILE"
  assert_success

  # ....Set main readme file to NorLab.............................................................
  cd "${BATS_DOCKER_WORKDIR}" || exit 1

  assert_file_exist NORLAB_PROJECT_TEMPLATE_INSTRUCTIONS.md
  assert_file_exist README.md
  assert_file_not_exist README.norlab_template.md
  assert_file_not_exist README.vaul_template.md

  assert_file_contains README.md "src=\"/visual/norlab_logo_acronym_dark.png"
  assert_file_exist visual/norlab_logo_acronym_dark.png
  assert_file_exist visual/norlab_logo_acronym_light.png

  assert_file_contains README.md "img.shields.io/github/v/release/norlab-ulaval/template-norlab-project"
  assert_file_contains README.md "norlab-ulaval/template-norlab-project.git"

}

@test "Case install VAUL readme  › expect pass" {
#  skip "tmp dev" # ToDo: on task end >> delete this line ←

  # Note: \n is to simulate the return key
  # Install NBS › Y
  # Install N2ST › Y
  # Semantic-Release › Y
  # Project env var prefix › TMP1
  # Install VAUL readme › V
  local TEST_CASE="YYYTMP1\nV"

  norlab_project_template_directory_reset_check

  # ....Execute initialize_norlab_project_template.bash............................................
  run bash -c "echo -e \"${TEST_CASE}\" | bash ./$TESTED_FILE"
  assert_success

  # ....Set main readme file to VAUL...............................................................

  cd "${BATS_DOCKER_WORKDIR}" || exit 1
  assert_file_exist NORLAB_PROJECT_TEMPLATE_INSTRUCTIONS.md
  assert_file_exist README.md
  assert_file_not_exist README.norlab_template.md
  assert_file_not_exist README.vaul_template.md

  assert_file_contains README.md "img src=\"./visual/VAUL_Logo_patch.png"
  assert_file_exist visual/VAUL_Logo_patch.png

  assert_file_contains README.md "img.shields.io/github/v/release/vaul-ulaval/template-norlab-project"
  assert_file_contains README.md "vaul-ulaval/template-norlab-project.git"

}



