#!/bin/bash
DOCUMENTATION_RUN_SLURM=$( cat <<'EOF'
# =================================================================================================
# Convenient script for running container specified in docker-compose.project.run.slurm.yaml and
#  docker-compose.project.build.native.yaml.
# Handle stoping the container in case the slurm command `scancel` is issued.
#
# Usage:
#   $ bash run.slurm.bash <sjob-id> [<optional-flag>] [--] [<any-python-arg>]
#
# Optional flag:
#   --log-name=<name>                                 The log file name without postfix
#   --log-path=<absolute-path-super-project-root>     The Absolute path to the slurm log directory.
#                                                     Will be created if it does not exist.
#   --skip-core-force-rebuild
#   --hydra-dry-run                                         Dry-run slurm job using registered hydra flag
#   --register-hydra-dry-run-flag                      Hydra flag used by '--hydra-dry-run'
#                                                     e.g., "+dev@_global_=math_env_slurm_job_dryrun"
#   -h | --help
#
# Positional argument:
#   <sjob-id>              (required) Used to ID the docker container, slurm job, optuna study ...
#   <any-docker-flag>       Any docker flag (optional)
#
# =================================================================================================
EOF
)

# (Priority) ToDo: unit-test for flag option

pushd "$(pwd)" >/dev/null || exit 1

# .................................................................................................
function show_help() {
  # (NICE TO HAVE) ToDo: refactor as a n2st fct (ref NMO-583)
  echo -e "${MSG_DIMMED_FORMAT}"
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "$0 --help\n"
  # Strip shell comment char `#` and both lines
  echo -e "${DOCUMENTATION_RUN_SLURM}" | sed 's/\# ====.*//' | sed 's/^\#//'
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "${MSG_END_FORMAT}"
}

function execute_on_exit() {
#  COMPOSE_PATH="${SUPER_PROJECT_ROOT:?err}/.dockerized_norlab_project/configuration"
  COMPOSE_PATH="${DNP_ROOT:?err}/src/lib/core/docker"
  THE_COMPOSE_FILE=docker-compose.project.run.slurm.yaml
  RUNNING_CONTAINER_IDS=$(docker compose -f "${COMPOSE_PATH}/${THE_COMPOSE_FILE}" ps --quiet --all --orphans=false)

  for each_id in "${RUNNING_CONTAINER_IDS[@]}"; do
    if [[ "${CONTAINER_ID}" == "${each_id}" ]]; then
      n2st::print_msg "Stoping container ${CONTAINER_ID}"
      docker stop "${CONTAINER_ID}" 2>/dev/null
    fi
  done

  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  popd >/dev/null || exit 1
  exit ${EXIT_CODE:1}
}


# ....Set env variables (pre cli)..................................................................
declare -a PYTHON_ARG
declare -a REGISTERED_HYDRA_DRYRUN_FLAG
declare -x SJOB_ID
declare -x IS_SLURM_RUN
declare LOG_NAME
declare LOG_PATH
declare DRY_RUN_SLURM_JOB

SJOB_ID="$1"
shift # Remove SJOB_ID argument value

LOG_NAME="slurm_job"
FORCE_REBUILD_PROJECT_CORE=true
FORCE_REBUILD_SLURM_IMG=true
DRY_RUN_SLURM_JOB=false


# ....cli..........................................................................................
while [ $# -gt 0 ]; do

  case $1 in
    --register-hydra-dry-run-flag)
      REGISTERED_HYDRA_DRYRUN_FLAG+=("${2}")
      shift # Remove argument (--register-hydra-dry-run-flag)
      shift # Remove argument value
      ;;
    --hydra-dry-run)
      DRY_RUN_SLURM_JOB=true
      shift # Remove argument (--hydra-dry-run)
      ;;
    --log-name)
      LOG_NAME="${2}"
      shift # Remove argument (--log-name)
      shift # Remove argument value
      ;;
    --log-path)
      LOG_PATH="${2}"
      shift # Remove argument (--log-path)
      shift # Remove argument value
      ;;
    --skip-core-force-rebuild)
      FORCE_REBUILD_PROJECT_CORE=false
      shift # Remove argument (--skip-force-rebuild)
      ;;
    -h | --help)
      clear
      show_help
      exit
      ;;
    --) # no more option
      shift
      PYTHON_ARG=( "$@" )
      break
      ;;
    *) # Default case
      PYTHON_ARG=("$@")
      break
      ;;
  esac

done

# ....Sanity check.................................................................................
test -n "${SJOB_ID}" || { echo "$(basename $0) | Positional argument <job-name> is empty" ; exit 1 ; }
test -n "${PYTHON_ARG[0]}" || { echo "$(basename $0) | Positional argument <any-python-arg> is empty" ; exit 1 ; }

# ....Set env variables (post cli).................................................................
DN_PROJECT_CONFIG_DIR="${DNP_ROOT:?err}/src/lib/core/docker"
COMPOSE_FILE="docker-compose.project.run.slurm.yaml"
COMPOSE_FILE_PATH=${DN_PROJECT_CONFIG_DIR}/${COMPOSE_FILE}

# ....Source project shell-scripts dependencies..................................................
SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
SCRIPT_PATH_PARENT="$(dirname "${SCRIPT_PATH}")"
source "${SCRIPT_PATH_PARENT}/../utils/import_dnp_lib.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/../utils/load_super_project_config.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/../utils/execute_compose.bash" || exit 1


# ====Begin========================================================================================
n2st::norlab_splash "${PROJECT_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"

if [[ $(uname -s) == "Darwin" ]] || [[ $(nvcc -V | grep 'nvcc: NVIDIA (R) Cuda compiler driver') != "nvcc: NVIDIA (R) Cuda compiler driver" ]]; then
  n2st::print_msg_warning "Host computer does not support nvidia gpu, changing container runtime to docker default."
  THE_SERVICE="project-slurm-no-gpu"
else
  THE_SERVICE="project-slurm"
fi

n2st::print_msg "FORCE_REBUILD_PROJECT_CORE: ${FORCE_REBUILD_PROJECT_CORE}, FORCE_REBUILD_SLURM_IMG: ${FORCE_REBUILD_SLURM_IMG}"

# ....Build image in the local store...............................................................
if [[ ${FORCE_REBUILD_PROJECT_CORE} == true ]]; then
  # shellcheck disable=SC2034
  ADD_DOCKER_FLAG=("--no-cache" "project-core")
  ADD_DOCKER_FLAG=("--quiet")
  dnp::excute_compose_on_dn_project_image "${ADD_DOCKER_FLAG[@]}"
fi

if [[ ${FORCE_REBUILD_SLURM_IMG} == true ]]; then
  DOCKER_BUILD=()
  DOCKER_BUILD+=("--no-cache")
  DOCKER_BUILD+=("--quiet")
  ADD_DOCKER_FLAG=("${DOCKER_BUILD[@]}" "${THE_SERVICE}")
  dnp::excute_compose_on_dn_project_image "${ADD_DOCKER_FLAG[@]}"
fi

cd "${SUPER_PROJECT_ROOT:?err}" || exit 1

# ....Set environment variable for compose project.................................................
export SJOB_ID
export IS_SLURM_RUN=true

# ....Run container on MAMBA/SLURM.................................................................
COMPOSE_FLAGS=("-f" "${COMPOSE_FILE_PATH}")

DOCKER_RUN=("run" "--rm")
DOCKER_RUN+=("--name=${DN_CONTAINER_NAME:?err}-slurm-${SJOB_ID}")
#DOCKER_RUN+=("--service-ports") # Publish compose service ports (Mute if collision with host)

if [[ ${DRY_RUN_SLURM_JOB} == true ]]; then
  PYTHON_ARG+=( "${REGISTERED_HYDRA_DRYRUN_FLAG[@]}" )
  DOCKER_RUN+=("${THE_SERVICE}" "${PYTHON_ARG[@]}")
  n2st::print_msg "Dryrun execute docker compose ${COMPOSE_FLAGS[*]} ${DOCKER_RUN[*]}\n"
  docker compose "${COMPOSE_FLAGS[@]}" "${DOCKER_RUN[@]}"
  EXIT_CODE=$?
else

  DOCKER_RUN+=("--detach") # Return the container ID
  DOCKER_RUN+=("${THE_SERVICE}" "${PYTHON_ARG[@]}")
  n2st::print_msg "Execute docker compose ${COMPOSE_FLAGS[*]} ${DOCKER_RUN[*]}\n"

  CONTAINER_ID=$(
    docker compose "${COMPOSE_FLAGS[@]}" "${DOCKER_RUN[@]}"
  )
  trap execute_on_exit EXIT
  n2st::print_msg "CONTAINER_ID=${CONTAINER_ID}\n" # Require `--detach` flag

  if [[ -n "${LOG_PATH}" ]]; then
    mkdir -p "${SUPER_PROJECT_ROOT:?err}/${LOG_PATH}"
    touch "${SUPER_PROJECT_ROOT:?err}/${LOG_PATH}/${LOG_NAME}.log"
    DOCKER_LOG=()
    DOCKER_LOG+=("logs" "--follow")
    DOCKER_LOG+=("--details")
    n2st::print_msg "Execute docker ${DOCKER_LOG[*]}"
    # Note: Operator "2>&1 |" redirect both stdin and stderr (portable version of "|&")
    docker "${DOCKER_LOG[@]}" "${CONTAINER_ID}" 2>&1 | tee "${SUPER_PROJECT_ROOT:?err}/${LOG_PATH}/${LOG_NAME}.log"
  fi

  echo && n2st::print_msg "Execute docker wait"
  docker wait "${CONTAINER_ID}" # Required if docker logs is skipped
  EXIT_CODE=$?
fi

# ====Teardown=====================================================================================
# The teardown logic of this script is handled by the trap function `execute_on_exit`
exit $EXIT_CODE
