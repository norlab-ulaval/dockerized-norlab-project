#!/bin/bash
DOCUMENTATION_RUN_SLURM=$( cat <<'EOF'
# =================================================================================================
# Convenient script for building and running container specified in
# docker-compose.project.run.slurm.yaml and docker-compose.project.build.native.yaml.
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
# Globals:
#   write SJOB_ID
#   write IS_SLURM_RUN
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
  echo -e "$0 --help"
  # Strip shell comment char `#` and both lines
  echo -e "${DOCUMENTATION_RUN_SLURM}" | sed 's/\# ====.*//' | sed 's/^\#//'
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "${MSG_END_FORMAT}"
}

function execute_on_exit() {
#  compose_path="${SUPER_PROJECT_ROOT:?err}/.dockerized_norlab_project/configuration"
  compose_path="${DNP_ROOT:?err}/src/lib/core/docker"
  the_compose_file=docker-compose.project.run.slurm.yaml
  running_container_ids=$(docker compose -f "${compose_path}/${the_compose_file}" ps --quiet --all --orphans=false)

  for each_id in "${running_container_ids[@]}"; do
    if [[ "${container_id}" == "${each_id}" ]]; then
      n2st::print_msg "Stoping container ${container_id}"
      docker stop "${container_id}" 2>/dev/null
    fi
  done

  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  popd >/dev/null || exit 1
  exit ${exit_code:1}
}


# ....Set env variables (pre cli)..................................................................
# Script locals env var
declare -a python_arg
declare -a registered_hydra_dryrun_flag
declare log_name
declare log_path
declare dry_run_slurm_job

# Exported env var
declare -x SJOB_ID
declare -x IS_SLURM_RUN

# Positional argument
SJOB_ID="$1"
shift # Remove SJOB_ID argument value

# Default values
log_name="slurm_job"
force_rebuild_project_core=true
force_rebuild_slurm_img=true
dry_run_slurm_job=false


# ....cli..........................................................................................
while [ $# -gt 0 ]; do

  case $1 in
    --register-hydra-dry-run-flag)
      registered_hydra_dryrun_flag+=("${2}")
      shift # Remove argument (--register-hydra-dry-run-flag)
      shift # Remove argument value
      ;;
    --hydra-dry-run)
      dry_run_slurm_job=true
      shift # Remove argument (--hydra-dry-run)
      ;;
    --log-name)
      log_name="${2}"
      shift # Remove argument (--log-name)
      shift # Remove argument value
      ;;
    --log-path)
      log_path="${2}"
      shift # Remove argument (--log-path)
      shift # Remove argument value
      ;;
    --skip-core-force-rebuild)
      force_rebuild_project_core=false
      shift # Remove argument (--skip-force-rebuild)
      ;;
    -h | --help)
      clear
      show_help
      exit
      ;;
    --) # no more option
      shift
      python_arg=( "$@" )
      break
      ;;
    *) # Default case
      python_arg=("$@")
      break
      ;;
  esac

done

# ....Sanity check.................................................................................
test -n "${SJOB_ID}" || { echo "$(basename $0) | Positional argument <job-name> is empty" ; exit 1 ; }
test -n "${python_arg[0]}" || { echo "$(basename $0) | Positional argument <any-python-arg> is empty" ; exit 1 ; }

# ....Set env variables (post cli).................................................................
dn_project_config_dir="${DNP_ROOT:?err}/src/lib/core/docker"
compose_file="docker-compose.project.run.slurm.yaml"
compose_file_path=${dn_project_config_dir}/${compose_file}

# ....Source project shell-scripts dependencies..................................................
if [[ -z ${DNP_ROOT}  ]] || [[ -z ${SUPER_PROJECT_ROOT}  ]]; then
  script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  script_path_parent="$(dirname "${script_path}")"
  source "${script_path_parent}/../utils/import_dnp_lib.bash" || exit 1
  source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1
  source "${script_path_parent}/../utils/execute_compose.bash" || exit 1
fi


# ====Begin========================================================================================
n2st::norlab_splash "${PROJECT_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"

if [[ $(uname -s) == "Darwin" ]] || [[ $(nvcc -V | grep 'nvcc: NVIDIA (R) Cuda compiler driver') != "nvcc: NVIDIA (R) Cuda compiler driver" ]]; then
  n2st::print_msg_warning "Host computer does not support nvidia gpu, changing container runtime to docker default."
  the_service="project-slurm-no-gpu"
else
  the_service="project-slurm"
fi

n2st::print_msg "force_rebuild_project_core: ${force_rebuild_project_core}, force_rebuild_slurm_img: ${force_rebuild_slurm_img}"

# ....Build image in the local store...............................................................
if [[ ${force_rebuild_project_core} == true ]]; then
  # shellcheck disable=SC2034
  add_docker_flag=("--no-cache" "project-core")
  add_docker_flag=("--quiet")
  dnp::excute_compose_on_dn_project_image "${add_docker_flag[@]}"
fi

if [[ ${force_rebuild_slurm_img} == true ]]; then
  docker_build=()
  docker_build+=("--no-cache")
  docker_build+=("--quiet")
  add_docker_flag=("${docker_build[@]}" "${the_service}")
  dnp::excute_compose_on_dn_project_image "${add_docker_flag[@]}"
fi

cd "${SUPER_PROJECT_ROOT:?err}" || exit 1

# ....Set environment variable for compose project.................................................
export SJOB_ID
export IS_SLURM_RUN=true

# ....Run container on MAMBA/SLURM.................................................................
compose_flags=("-f" "${compose_file_path}")

docker_run=("run" "--rm")
docker_run+=("--name=${DN_CONTAINER_NAME:?err}-slurm-${SJOB_ID}")
#docker_run+=("--service-ports") # Publish compose service ports (Mute if collision with host)

if [[ ${dry_run_slurm_job} == true ]]; then
  python_arg+=( "${registered_hydra_dryrun_flag[@]}" )
  docker_run+=("${the_service}" "${python_arg[@]}")
  n2st::print_msg "Dryrun execute docker compose ${compose_flags[*]} ${docker_run[*]}\n"
  docker compose "${compose_flags[@]}" "${docker_run[@]}"
  exit_code=$?
else

  docker_run+=("--detach") # Return the container ID
  docker_run+=("${the_service}" "${python_arg[@]}")
  n2st::print_msg "Execute docker compose ${compose_flags[*]} ${docker_run[*]}\n"

  container_id=$(
    docker compose "${compose_flags[@]}" "${docker_run[@]}"
  )
  trap execute_on_exit EXIT
  n2st::print_msg "container_id=${container_id}\n" # Require `--detach` flag

  if [[ -n "${log_path}" ]]; then
    mkdir -p "${SUPER_PROJECT_ROOT:?err}/${log_path}"
    touch "${SUPER_PROJECT_ROOT:?err}/${log_path}/${log_name}.log"
    docker_log=()
    docker_log+=("logs" "--follow")
    docker_log+=("--details")
    n2st::print_msg "Execute docker ${docker_log[*]}"
    # Note: Operator "2>&1 |" redirect both stdin and stderr (portable version of "|&")
    docker "${docker_log[@]}" "${container_id}" 2>&1 | tee "${SUPER_PROJECT_ROOT:?err}/${log_path}/${log_name}.log"
  fi

  echo && n2st::print_msg "Execute docker wait"
  docker wait "${container_id}" # Required if docker logs is skipped
  exit_code=$?
fi

# ====Teardown=====================================================================================
# The teardown logic of this script is handled by the trap function `execute_on_exit`
exit $exit_code
