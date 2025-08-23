#!/bin/bash
DOCUMENTATION_RUN_SLURM=$( cat <<'EOF'
# =================================================================================================
# Convenient script for building and running container specified in
# docker-compose.project.run.slurm.yaml and docker-compose.project.build.native.yaml.
# Handle stoping the container in case the slurm command `scancel` is issued.
#
# Usage:
#   $ bash run.slurm.bash <sjob-id> [<optional-flag>] [--] <any-python-args>
#
# Optional flag:
#   --log-name <name>                                 The log file name without postfix
#   --log-path <absolute-path-super-project-root>     The Absolute path to the slurm log directory.
#                                                     Will be created if it does not exist.
#   --skip-core-force-rebuild
#   --skip-slurm-force-rebuild
#   --hydra-dry-run                                   Dry-run slurm job using registered hydra flag
#   --register-hydra-dry-run-flag                     Hydra flag used by '--hydra-dry-run'
#                                                     e.g., "+dev@_global_=math_env_slurm_job_dryrun"
#   -h | --help
#
# Optional docker run flags:
#   -e, --env stringArray        Set container environment variables
#   -w, --workdir string         Override path to workdir directory
#   -T, --no-TTY                 Disable pseudo-TTY allocation
#   -v, --volume stringArray     Bind mount a volume
#
# Positional argument:
#   <sjob-id>              (required) Used to ID the docker container, slurm job, optuna study ...
#   <any-python-args>      (required) The python command with flags
#
# Globals:
#   read DNA_ROOT
#   read SUPER_PROJECT_ROOT
#   write SJOB_ID
#   write IS_SLURM_RUN
#
# =================================================================================================
EOF
)
# (Priority) ToDo: add missing flag options unit-test


# ....Script setup.................................................................................
# Exported env var
declare -x SJOB_ID
declare -x IS_SLURM_RUN

pushd "$(pwd)" >/dev/null || exit 1
# Note: Keep the pushd/popd logic for now

# .................................................................................................
function dna::show_help() {
  # (NICE TO HAVE) ToDo: refactor as a n2st fct (ref NMO-583)
  echo -e "${MSG_DIMMED_FORMAT}"
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "$0 --help"
  # Strip shell comment char `#` and both lines
  echo -e "${DOCUMENTATION_RUN_SLURM}" | sed '/\# ====.*/d' | sed 's/^\# //' | sed 's/^\#//'
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "${MSG_END_FORMAT}"
}

function dna::run_slurm_teardown_callback() {
  exit_code=$?
  if [[ ${exit_code} -ne 0 ]]; then
    n2st::print_msg_error "Container exited with error ${exit_code}"
  fi
  local compose_path="${DNA_ROOT:?err}/src/lib/core/docker"
  local the_compose_file=docker-compose.project.run.slurm.yaml
  local running_container_ids
  running_container_ids=$(docker compose -f "${compose_path}/${the_compose_file}" ps --quiet --all --orphans=false)
  if [[ -n ${running_container_ids} ]]; then
    for each_id in "${running_container_ids[@]}"; do
      if [[ "${container_id}" == "${each_id}" ]]; then
        n2st::print_msg "Stoping container ${container_id}"
        docker stop "${container_id}" 2>/dev/null
      fi
    done
  fi

  popd >/dev/null || { echo "Return to original dir error" 1>&2 && exit 1; }
  # Note: Keep the pushd/popd logic for now

  return ${exit_code:-1}
}

function dna::run_slurm() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  # Positional argument
  SJOB_ID="$1"
  shift # Remove SJOB_ID argument value

  # ....Pre-condition..............................................................................

  # ....Set env variables (pre cli)................................................................
  # Script locals env var
  declare -a python_arg=()
  declare -a registered_hydra_dryrun_flag=()
  declare log_name
  declare log_path
  declare dry_run_slurm_job
  declare -a docker_run_args

  # Default values
  log_name="slurm_job"
  force_rebuild_project_core=true
  force_rebuild_slurm_img=true
  dry_run_slurm_job=false
  docker_run_args=()

  if [[ "${SJOB_ID}" == "--help"  ]] || [[ "${SJOB_ID}" == "-h"  ]]; then
    dna::show_help
    exit
  fi


  # ....cli........................................................................................
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
      --skip-slurm-force-rebuild)
        force_rebuild_slurm_img=false
        shift # Remove argument (--skip-slurm-force-rebuild)
        ;;
      -e|--env|-w|--workdir|-v|--volume) # Assume its a docker compose flag
        docker_run_args+=("$1" "$2")
        shift
        shift
        ;;
      -h | --help)
        clear
        dna::show_help
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

  # ....Sanity check...............................................................................

  test -n "${SJOB_ID}" || n2st::print_msg_error_and_exit "Missing sjob-id mandatory positional argument!"
  test -n "${python_arg[0]}" || n2st::print_msg_error_and_exit "Missing <any-python-arg> mandatory positional argument!"

  # ....Set env variables (post cli)...............................................................
  local compose_path="${DNA_ROOT:?err}/src/lib/core/docker"
  local compose_file="docker-compose.project.run.slurm.yaml"
  local compose_file_path=${compose_path}/${compose_file}
  local the_service="project-slurm"


  # ====Begin======================================================================================
  n2st::print_msg "force_rebuild_project_core: ${force_rebuild_project_core}, force_rebuild_slurm_img: ${force_rebuild_slurm_img}"

  # ....Build image in the local store.............................................................
  if [[ ${force_rebuild_project_core} == true ]]; then
    # shellcheck disable=SC2034
    declare -a docker_build=()
    docker_build+=("--service-names" "project-core-pre,project-core-user,project-core")
    docker_build+=("--" "--no-cache")
    dna::build_services "${docker_build[@]}"  || exit 1
  fi

  if [[ ${force_rebuild_slurm_img} == true ]]; then
    declare -a docker_build=()
    docker_build+=("--service-names" "${the_service}")
    docker_build+=("--" "--no-cache")
    dna::build_services "${docker_build[@]}"  || exit 1
  fi

  # ....Set GPU capabilities.......................................................................
  dna::configure_gpu_capabilities "$(n2st::which_architecture_and_os)" "${compose_path}" "${compose_file}" "${the_service}" || n2st::print_msg_error_and_exit "dna::configure_gpu_capabilities failed!"
  test -n "${NVIDIA_VISIBLE_DEVICES:?'Env variable need to be set and non-empty.'}"
  test -n "${NVIDIA_DRIVER_CAPABILITIES}" # Might be empty or unset -> default driver capability: utility, compute
  test -n "${DN_DOCKER_RUNTIME:?'Env variable need to be set and non-empty.'}"
  test -n "${DN_HOST_GPU_ARCHITECTURE:?'Env variable need to be set and non-empty.'}"

  # ....Set environment variable for compose project...............................................
  export SJOB_ID

  # (☕minor) ToDo: validate deleting env var IS_SLURM_RUN -> its not used
  export IS_SLURM_RUN=true

  # ....Run container on MAMBA/SLURM...............................................................
  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1

  compose_flags=("-f" "${compose_file_path}")

  declare -a docker_run=()
  docker_run+=("run" "--rm")
  docker_run+=("${docker_run_args[@]}")
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
    n2st::print_msg "Execute docker compose ${MSG_DIMMED_FORMAT}${compose_flags[*]} ${docker_run[*]}${MSG_END_FORMAT}\n"

    container_id=$(
      docker compose "${compose_flags[@]}" "${docker_run[@]}"
    )
    trap dna::run_slurm_teardown_callback EXIT
    n2st::print_msg "container_id=${container_id}\n" # Require `--detach` flag

    if [[ -n "${log_path}" ]]; then
      mkdir -p "${SUPER_PROJECT_ROOT:?err}/${log_path}"
      touch "${SUPER_PROJECT_ROOT:?err}/${log_path}/${log_name}.log"
      docker_log=()
      docker_log+=("logs" "--follow")
      docker_log+=("--details")
      n2st::print_msg "Execute docker ${MSG_DIMMED_FORMAT}${docker_log[*]}${MSG_END_FORMAT}"
      # Note: Operator "2>&1 |" redirect both stdin and stderr (portable version of "|&")
      docker "${docker_log[@]}" "${container_id}" 2>&1 | tee "${SUPER_PROJECT_ROOT:?err}/${log_path}/${log_name}.log"
    fi

    echo && n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}docker wait${MSG_END_FORMAT}"
    docker wait "${container_id}" # Required if docker logs is skipped
    exit_code=$?
  fi

  # ....Teardown...................................................................................
  # The teardown logic of this script is handled by the trap function `dna::run_slurm_teardown_callback`
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return $exit_code
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dna_error_prefix="\033[1;31m[dna error]\033[0m"
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath -q "${BASH_SOURCE[0]:-.}")"
  script_path_parent="$(dirname "${script_path}")"
  if [[ -z $( declare -f dna::import_lib_and_dependencies ) ]]; then
    source "${script_path_parent}/../utils/import_dna_lib.bash" || exit 1
    source "${script_path_parent}/../utils/execute_compose.bash" || exit 1
    source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1
    source "${script_path_parent}/build.all.bash" || return 1
  fi

  # ....Execute....................................................................................
  if [[ "${DNA_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
    clear
  fi
  n2st::norlab_splash "${DNA_SPLASH_NAME_FULL:?err}" "${DNA_GIT_REMOTE_URL}" "negative"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dna::run_slurm "$@"
  fct_exit_code=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${fct_exit_code}"
else
  # This script is being sourced, ie: __name__="__source__"

  # ....Pre-condition..............................................................................
  test -n "$( declare -f dna::import_lib_and_dependencies )" || { echo -e "${dna_error_prefix} The DNA lib is not loaded!" 1>&2 && exit 1; }
  test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
  test -n "${SUPER_PROJECT_ROOT}" || { echo -e "${dna_error_prefix} The super project DNA configuration is not loaded!" 1>&2 && exit 1; }
fi
