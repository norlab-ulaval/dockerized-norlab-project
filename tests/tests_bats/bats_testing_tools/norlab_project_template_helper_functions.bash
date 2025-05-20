
# ====Norlab Project Template======================================================================

function norlab_project_template_directory_reset_check() {

    # ....Check project root directories...........................................................
    assert_dir_exist "/code/template-norlab-project"
    assert_dir_exist "${TEST_TEMP_DIR}/template-norlab-project"
    assert_equal $(pwd) "${TEST_TEMP_DIR}/template-norlab-project"

    # ....Check git related........................................................................
    assert_dir_exist .git
    assert_file_exist .gitmodules
    assert_file_exist .gitignore
    assert_file_exist commit_msg_reference.md
    assert_file_exist .github/CODEOWNERS
    assert_file_exist .github/pull_request_template.md

    # ....Check readme files.......................................................................
    assert_file_exist README.md
    assert_file_exist README.norlab_template.md
    assert_file_exist README.vaul_template.md

    # ....Check semantic-release related...........................................................
    assert_file_exist version.txt
    assert_file_exist CHANGELOG.md
    assert_file_exist .releaserc.json
    assert_file_exist .github/workflows/semantic_release.yml

    # ....Check tests related files/directory....................................................................
    assert_dir_exist tests/tests_bats/bats_testing_tools
    assert_file_exist tests/tests_bats/bats_testing_tools/bats_helper_functions_local.bash
    assert_file_exist tests/run_bats_core_test_in_n2st.template.bash
    assert_file_exist tests/run_bats_core_test_in_n2st.tnp.bash
    assert_file_exist tests/tests_bats/test_template.bats
    assert_file_exist src/dummy.bash
    assert_file_exist tests/tests_bats/bats_testing_tools/norlab_project_template_helper_functions.bash
    assert_file_exist tests/tests_bats/test_dotenv_files.bats
    assert_file_exist tests/tests_bats/test_initialize_norlab_project_template.bats

    # ....Check N2ST related.......................................................................
    assert_dir_exist utilities/norlab-shell-script-tools
    assert_file_contains .gitmodules "\[submodule \"utilities/norlab-shell-script-tools\"\]"
    assert_file_contains .gitmodules .*"url = https://github.com/norlab-ulaval/norlab-shell-script-tools.git"

    # ....Check NBS related........................................................................
    assert_dir_not_exist utilities/norlab-build-system
    assert_file_not_contains .gitmodules "\[submodule \"utilities/norlab-build-system\"\]"
    assert_file_not_contains .gitmodules "url = https://github.com/norlab-ulaval/norlab-build-system.git"

    # ....Check NorLab project template logic related..............................................
    assert_file_exist initialize_norlab_project_template.bash
    assert_file_exist .env.template-norlab-project.template

    # ....Check dotenv content.....................................................................
    assert_file_contains .env.template-norlab-project.template "^PROJECT_PROMPT_NAME='Norlab-Project-Template'"
    assert_file_contains .env.template-norlab-project.template "^#NBS_SPLASH_NAME=.*"
    assert_file_contains .env.template-norlab-project.template "^N2ST_PATH=\${PROJECT_PATH}/utilities/norlab-shell-script-tools"
    assert_file_contains .env.template-norlab-project.template "^#NBS_PATH=\${PROJECT_PATH}/utilities/norlab-build-system"
    assert_file_contains .env.template-norlab-project.template "^PLACEHOLDER_PROMPT_NAME.*"
    assert_file_contains .env.template-norlab-project.template "^PLACEHOLDER_GIT_REMOTE_URL.*"
    assert_file_contains .env.template-norlab-project.template "^PLACEHOLDER_GIT_NAME.*"
    assert_file_contains .env.template-norlab-project.template "^PLACEHOLDER_PATH.*"
    assert_file_contains .env.template-norlab-project.template "^PLACEHOLDER_SRC_NAME.*"
    assert_file_contains .env.template-norlab-project.template "^PROJECT_PROMPT_NAME='Norlab-Project-Template'"
}

function check_NBS_is_installed() {
    cd "${BATS_DOCKER_WORKDIR}" || exit 1
    assert_output --regexp .*"\[Norlab-Project-Template\]".*"Installing NBS"
    assert_dir_exist utilities/norlab-build-system
    assert_file_contains .env.template-norlab-project "^NBS_PATH=\${PROJECT_PATH}/utilities/norlab-build-system"
    assert_file_contains .env.template-norlab-project "^NBS_SPLASH_NAME=.*"
}

function check_NBS_not_installed() {
  cd "${BATS_DOCKER_WORKDIR}" || exit 1
  assert_output --regexp .*"\[Norlab-Project-Template\]".*"Skipping NBS install"
  assert_dir_not_exist utilities/norlab-build-system
  assert_file_not_contains .env.template-norlab-project "^#NBS_PATH=\${PROJECT_PATH}/utilities/norlab-build-system"
  assert_file_not_contains .env.template-norlab-project "^NBS_PATH=\${PROJECT_PATH}/utilities/norlab-build-system"
  assert_file_not_contains .env.template-norlab-project "^NBS_SPLASH_NAME=.*"
}

function check_N2ST_is_installed() {
  cd "${BATS_DOCKER_WORKDIR}" || exit 1
  assert_output --regexp .*"\[Norlab-Project-Template\]".*"Installing N2ST"
  assert_dir_exist utilities/norlab-shell-script-tools
  assert_file_contains .env.template-norlab-project "^N2ST_PATH=\${PROJECT_PATH}/utilities/norlab-shell-script-tools"
  assert_file_exist src/dummy.bash
  assert_file_exist tests/run_bats_core_test_in_n2st.bash
  assert_file_exist tests/tests_bats/bats_testing_tools/bats_helper_functions_local.bash
  assert_file_exist tests/tests_bats/test_template.bats

}

function check_N2ST_not_installed() {
  cd "${BATS_DOCKER_WORKDIR}" || exit 1
  assert_output --regexp .*"\[Norlab-Project-Template\]".*"Skipping N2ST install"
  assert_dir_not_exist utilities/norlab-shell-script-tools
  assert_file_not_contains .env.template-norlab-project "^N2ST_PATH=\${PROJECT_PATH}/utilities/norlab-shell-script-tools"

  assert_file_not_exist src/dummy.bash
  assert_dir_not_exist tests/tests_bats
  assert_file_not_exist tests/run_bats_core_test_in_n2st.bash
}

function check_semantic_release_is_installed() {
  cd "${BATS_DOCKER_WORKDIR}" || exit 1
  assert_output --regexp .*"\[Norlab-Project-Template\]".*"Installing Semantic-Release"
  assert_file_exist version.txt
  assert_file_exist .releaserc.json
  assert_file_exist .github/workflows/semantic_release.yml
  assert_file_exist CHANGELOG.md

  assert_file_empty CHANGELOG.md
}

function check_semantic_release_not_installed() {
  assert_output --regexp .*"\[Norlab-Project-Template\]".*"Skipping Semantic-Release install"
  cd "${BATS_DOCKER_WORKDIR}" || exit 1
  assert_file_not_exist version.txt
  assert_file_not_exist CHANGELOG.md
  assert_file_not_exist .releaserc.json
  assert_file_not_exist .github/workflows/semantic_release.yml
}

function check_norlab_project_template_teardown() {
  assert_output --regexp .*"\[Norlab-Project-Template\]".*"Teardown clean-up"

  assert_output --regexp .*"\[Norlab-Project-Template done\]".*"You can delete ".*"initialize_norlab_project_template.bash".*"and ".*"NORLAB_PROJECT_TEMPLATE_INSTRUCTIONS.md".*"when you are ready.".*"NorLab project remaining configuration steps:".*"-".*"✔ Step 1 › Generate the new repository".*"-".*"✔ Step 2 › Execute initialize_norlab_project_template.bash".*"-   Step 3 › Make it your own".*"https://github.com/norlab-ulaval/template-norlab-project/tree/main#step-3--make-it-your-own".*"-   Step 4 › Configure the GitHub repository settings".*"https://github.com/norlab-ulaval/template-norlab-project/tree/main#step-4--configure-the-github-repository-settings".*"-   Step 5 › Release automation: enable semantic versioning tools".*"https://github.com/norlab-ulaval/template-norlab-project/tree/main#step-5--enable-release-automation-tools-semantic-versioning".*"Completed"

  cd "${BATS_DOCKER_WORKDIR}" || exit 1
  assert_file_not_exist tests/run_bats_core_test_in_n2st.tnp.bash
  assert_file_not_exist tests/tests_bats/bats_testing_tools/norlab_project_template_helper_functions.bash
  assert_file_not_exist tests/tests_bats/test_dotenv_files.bats
  assert_file_not_exist tests/tests_bats/test_initialize_norlab_project_template.bats

}
