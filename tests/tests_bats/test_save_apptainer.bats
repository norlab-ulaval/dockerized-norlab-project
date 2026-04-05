#!/usr/bin/env bats
# =================================================================================================
# Unit tests for save.bash --apptainer extension
#
# Test cases:
# - --apptainer flag requires a profile argument
# - --apptainer flag only valid with SERVICE=slurm
# - slurm service can be saved without --apptainer (produces tar archive only)
# - save with --apptainer generates dna_tar_to_apptainer_sif_converter.sh
# - save with --apptainer updates metadata with Apptainer info
# - save with --apptainer validates profile env file exists
# - --squash works for slurm/develop/deploy services (with or without --apptainer)
#
# =================================================================================================
bats_path=/usr/lib/bats
error_prefix="[\033[1;31mN2ST ERROR\033[0m]"
if [[ -d ${bats_path} ]]; then
  load "${bats_path}/bats-support/load"
  load "${bats_path}/bats-assert/load"
  load "${bats_path}/bats-file/load"
  load "${SRC_CODE_PATH:?err}/${N2ST_BATS_TESTING_TOOLS_RELATIVE_PATH:?err}/bats_helper_functions"
  load "${SRC_CODE_PATH}/tests/tests_bats/bats_testing_tools/bats_helper_functions_local"
else
  echo -e "\n${error_prefix} $0 path to bats-core helper library unreachable at \"${bats_path}\"!"
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi
# ====Tests file configuration=====================================================================
TESTED_FILE="save.bash"
TESTED_FILE_PATH="src/lib/commands"
# ....Setup........................................................................................
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  export MOCK_DNA_DIR=$(temp_make)
  export MOCK_SAVE_DIR=$(temp_make)

  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils/"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/commands/"

  export MOCK_PROJECT_ROOT="${MOCK_DNA_DIR}/mock_project"
  mkdir -p "${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile"
  printf '%s\n' "DN_PROJECT_USER=testuser" "DN_PROJECT_PATH=/ros2_ws/src/test-project" \
    > "${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.valeria"
  echo "mock git" > "${MOCK_PROJECT_ROOT}/.git/config" 2>/dev/null || true

  cat > "${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash" << 'EOF'
#!/bin/bash
export MSG_DIMMED_FORMAT=""
export MSG_END_FORMAT=""
export MSG_LINE_CHAR_BUILDER_LVL2="-"
export MSG_LINE_STYLE_LVL2=""
export DNA_ROOT="${MOCK_DNA_DIR}"
export DNA_LIB_PATH="${MOCK_DNA_DIR}/src/lib"
export DNA_SPLASH_NAME_SMALL="DNA"

function dna::import_lib_and_dependencies() { return 0; }
function n2st::print_msg() { echo "MSG: $*"; return 0; }
function n2st::print_msg_error() { echo "ERROR: $*" >&2; return 0; }
function n2st::print_msg_done() { echo "DONE: $*"; return 0; }
function n2st::print_msg_warning() { echo "WARNING: $*"; return 0; }
function n2st::print_formated_script_header() { echo "HEADER: $*"; return 0; }
function n2st::print_formated_script_footer() { echo "FOOTER: $*"; return 0; }
function n2st::set_which_architecture_and_os() { export IMAGE_ARCH_AND_OS="linux/x86"; return 0; }
function dna::command_help_menu() { echo "HELP: $*"; return 0; }
function dna::illegal_command_msg() { echo "ILLEGAL: $*" >&2; return 1; }

for func in $(compgen -A function | grep -e dna:: -e n2st::); do
  export -f "${func}"
done
echo "[dna done] Mock import_dna_lib.bash loaded"
EOF

  cat > "${MOCK_DNA_DIR}/src/lib/core/utils/load_super_project_config.bash" << EOF
#!/bin/bash
export SUPER_PROJECT_REPO_NAME="test-project"
export DN_PROJECT_IMAGE_NAME="test-image"
export DN_PROJECT_HUB="norlabulaval"
export PROJECT_TAG="l4t-r36.4.0"
export SUPER_PROJECT_ROOT="${MOCK_PROJECT_ROOT}"
export DN_PROJECT_GIT_REMOTE_URL="https://github.com/test/test-project.git"
export DN_PROJECT_ALIAS_PREFIX="test"
export DNA_CONFIG_SCHEME_VERSION=3
echo "Mock load_super_project_config.bash loaded"
return 0
EOF

  # Create mock apptainer_tools.bash
  cat > "${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash" << 'EOF'
#!/bin/bash
function dna::check_apptainer_profile_env_file() {
  local profile="$1"
  local env_file="${SUPER_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.${profile}"
  if [[ ! -f "${env_file}" ]]; then
    echo "ERROR: Profile env file not found: ${env_file}" >&2
    return 1
  fi
  return 0
}
function dna::generate_apptainer_build_sif_script() {
  local tar_filename="$1"
  local sif_name="$2"
  local output_dir="$3"
  echo "#!/bin/bash" > "${output_dir}/dna_tar_to_apptainer_sif_converter.sh"
  echo "apptainer build ${sif_name} docker-archive:${tar_filename}" >> "${output_dir}/dna_tar_to_apptainer_sif_converter.sh"
  chmod +x "${output_dir}/dna_tar_to_apptainer_sif_converter.sh"
  echo "DONE: Generated ${output_dir}/dna_tar_to_apptainer_sif_converter.sh"
  return 0
}
function dna::load_apptainer_profile_env() {
  local profile="$1"
  local profile_env_file="${SUPER_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.${profile}"
  if [[ ! -f "${profile_env_file}" ]]; then
    echo "ERROR: Profile env file not found: ${profile_env_file}" >&2
    return 1
  fi
  set -o allexport
  source "${profile_env_file}" || { set +o allexport; return 1; }
  set +o allexport
  if [[ -z "${DN_PROJECT_USER:-}" ]] || [[ "${DN_PROJECT_USER}" == "PLACEHOLDER_HPC_USERNAME" ]]; then
    echo "ERROR: DN_PROJECT_USER is not configured in ${profile_env_file}" >&2
    return 1
  fi
  echo "MSG: Using HPC server DN_PROJECT_USER=${DN_PROJECT_USER} (from profile: ${profile})"
  export DN_PROJECT_USER
  return 0
}
function dna::squash_docker_image() {
  local image_name="$1"
  echo "MSG: Mock dna::squash_docker_image called with image: ${image_name}"
  if [[ "${MOCK_SQUASH_FAIL:-false}" == "true" ]]; then
    echo "ERROR: Mock squash failure" >&2
    return 1
  fi
  return 0
}
for func in $(compgen -A function | grep -e dna::); do export -f "${func}"; done
EOF
}

setup() {
  mkdir -p "${MOCK_DNA_DIR}/src/lib/commands"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils"
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" "${MOCK_DNA_DIR}/src/lib/commands/"
  source "${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash" || exit 1
  cd "${MOCK_DNA_DIR}" || exit 1

  function docker() {
    case "$1" in
      "image")
        case "$2" in
          "save")
            # Parse --output argument regardless of flag order (e.g. --platform may precede --output)
            local _output_file=""
            local _i
            for (( _i=3; _i<=$#; _i++ )); do
              if [[ "${!_i}" == "--output" ]]; then
                local _next=$(( _i + 1 ))
                _output_file="${!_next}"
                break
              fi
            done
            [[ -n "${_output_file}" ]] && touch "${_output_file}"
            echo "Mock docker image save: $*"
            return 0 ;;
          *) echo "Mock docker image: $*"; return 0 ;;
        esac ;;
      *) echo "Mock docker: $*"; return 0 ;;
    esac
  }
  export -f docker

  function gzip() {
    echo "Mock gzip called with args: $*"
    # Simulate gzip behaviour: rename the file with .gz extension
    for arg in "$@"; do
      if [[ "${arg}" != -* && -f "${arg}" ]]; then
        mv "${arg}" "${arg}.gz"
      fi
    done
    return 0
  }
  export -f gzip

  function git() {
    case "$1" in
      "branch") echo "main"; return 0 ;;
      "rev-parse") echo "abc123"; return 0 ;;
      *) echo "Mock git: $*"; return 0 ;;
    esac
  }
  export -f git

  function date() {
    [[ "$1" == "+%Y%m%d%H%M" ]] && echo "202312151430" || echo "Fri Dec 15 14:30:00 UTC 2023"
  }
  export -f date
}

teardown() {
  bats_print_run_env_variable_on_error
}

teardown_file() {
  temp_del "${MOCK_DNA_DIR}"
  temp_del "${MOCK_SAVE_DIR}"
}

# ====Tests: --apptainer flag validation===========================================================

@test "dna::save_command --apptainer without profile argument › expect error" {
  run bash -c "source ${MOCK_DNA_DIR}/src/lib/commands/save.bash && dna::save_command --apptainer"
  assert_failure
  assert_output --partial "profile"
}

@test "dna::save_command slurm without --apptainer flag › expect success and saves tar archive only" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command ${MOCK_SAVE_DIR} slurm
  "
  assert_success
  assert_output --partial "save slurm image procedure"
}

@test "dna::save_command --apptainer with deploy service › expect error" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --apptainer valeria ${MOCK_SAVE_DIR} deploy
  "
  assert_failure
  assert_output --partial "--apptainer flag can only be used with SERVICE=slurm"
}

@test "dna::save_command --apptainer with develop service › expect error" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --apptainer valeria ${MOCK_SAVE_DIR} develop
  "
  assert_failure
  assert_output --partial "--apptainer flag can only be used with SERVICE=slurm"
}

# ====Tests: --apptainer slurm save================================================================

@test "dna::save_command --apptainer valeria slurm › expect success and creates compressed tar.gz" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --apptainer valeria ${MOCK_SAVE_DIR} slurm
  "
  assert_success
  assert_output --partial "Compressing tar archive"
  assert_output --partial "Mock gzip called with args:"
}

@test "dna::save_command --apptainer valeria slurm › uses --platform linux/amd64 for docker image save" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --apptainer valeria ${MOCK_SAVE_DIR} slurm
  "
  assert_success
  assert_output --partial "Mock docker image save: image save --platform linux/amd64"
}

@test "dna::save_command --apptainer valeria slurm with APPTAINER_TARGET_PLATFORM set › uses custom platform for docker image save" {
  run bash -c "
    export APPTAINER_TARGET_PLATFORM='linux/arm64'
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --apptainer valeria ${MOCK_SAVE_DIR} slurm
  "
  assert_success
  assert_output --partial "Mock docker image save: image save --platform linux/arm64"
}

@test "dna::save_command --apptainer valeria slurm › creates dna_tar_to_apptainer_sif_converter.sh" {
  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --apptainer valeria ${MOCK_SAVE_DIR} slurm
  "
  run find "${MOCK_SAVE_DIR}" -name "dna_tar_to_apptainer_sif_converter.sh"
  assert_success
  assert_output --partial "dna_tar_to_apptainer_sif_converter.sh"
}

@test "dna::save_command --apptainer valeria slurm › metadata contains APPTAINER_PROFILE" {
  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --apptainer valeria ${MOCK_SAVE_DIR} slurm
  "
  run find "${MOCK_SAVE_DIR}" -name "meta.txt" -exec grep "APPTAINER_PROFILE" {} \;
  assert_success
  assert_output --partial "valeria"
}

@test "dna::save_command --apptainer valeria slurm › metadata contains SIF_BUILD_CMD" {
  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --apptainer valeria ${MOCK_SAVE_DIR} slurm
  "
  run find "${MOCK_SAVE_DIR}" -name "meta.txt" -exec grep "SIF_BUILD_CMD" {} \;
  assert_success
  assert_output --partial "apptainer build"
}

@test "dna::save_command --apptainer valeria slurm › metadata TAR_FILENAME references .tar.gz archive" {
  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --apptainer valeria ${MOCK_SAVE_DIR} slurm
  "
  run find "${MOCK_SAVE_DIR}" -name "meta.txt" -exec grep "TAR_FILENAME" {} \;
  assert_success
  assert_output --partial ".tar.gz"
}

@test "dna::save_command --apptainer valeria slurm › metadata contains linux/amd64 platform" {
  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --apptainer valeria ${MOCK_SAVE_DIR} slurm
  "
  run find "${MOCK_SAVE_DIR}" -name "meta.txt" -exec grep "APPTAINER_TARGET_PLATFORM" {} \;
  assert_success
  assert_output --partial "linux/amd64"
}

@test "dna::save_command --apptainer with missing profile env file › expect error" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --apptainer nonexistent_profile ${MOCK_SAVE_DIR} slurm
  "
  assert_failure
  assert_output --partial "not found"
}

# ====Tests: --squash flag validation==============================================================

@test "dna::save_command --squash slurm without --apptainer › expect success and squashes slurm image" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --squash ${MOCK_SAVE_DIR} slurm
  "
  assert_success
  assert_output --partial "Mock dna::squash_docker_image called with image:"
}

# ====Tests: --squash with --apptainer slurm save=================================================

@test "dna::save_command --squash --apptainer valeria slurm › expect success" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --squash --apptainer valeria ${MOCK_SAVE_DIR} slurm
  "
  assert_success
}

@test "dna::save_command --squash --apptainer valeria slurm › calls dna::squash_docker_image" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --squash --apptainer valeria ${MOCK_SAVE_DIR} slurm
  "
  assert_success
  assert_output --partial "Mock dna::squash_docker_image called with image:"
}

@test "dna::save_command --squash --apptainer valeria slurm › creates dna_tar_to_apptainer_sif_converter.sh" {
  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --squash --apptainer valeria ${MOCK_SAVE_DIR} slurm
  "
  run find "${MOCK_SAVE_DIR}" -name "dna_tar_to_apptainer_sif_converter.sh"
  assert_success
  assert_output --partial "dna_tar_to_apptainer_sif_converter.sh"
}

@test "dna::save_command --squash --apptainer valeria slurm when squash fails › expect error" {
  run bash -c "
    export MOCK_SQUASH_FAIL=true
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --squash --apptainer valeria ${MOCK_SAVE_DIR} slurm
  "
  assert_failure
  assert_output --partial "Failed to squash Docker image"
}

# ====Tests: --squash with develop/deploy (without --apptainer)===================================

@test "dna::save_command --squash develop › expect success and calls dna::squash_docker_image" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --squash ${MOCK_SAVE_DIR} develop
  "
  assert_success
  assert_output --partial "Mock dna::squash_docker_image called with image:"
}

@test "dna::save_command --squash deploy › expect success and calls dna::squash_docker_image" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --squash ${MOCK_SAVE_DIR} deploy
  "
  assert_success
  assert_output --partial "Mock dna::squash_docker_image called with image:"
}

@test "dna::save_command --squash develop when squash fails › expect error" {
  run bash -c "
    export MOCK_SQUASH_FAIL=true
    source ${MOCK_DNA_DIR}/src/lib/commands/save.bash
    dna::save_command --squash ${MOCK_SAVE_DIR} develop
  "
  assert_failure
  assert_output --partial "Failed to squash Docker image"
}
