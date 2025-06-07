#!/bin/bash
# =================================================================================================
# Kill docker compose ressources alocated by run.slurm.bash
#
# Usage:
#   $ bash down.slurm.bash [<any-docker-compose-down-flags>]
#
# =================================================================================================
remaining_args=("$@")

if [[ "${DNP_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
  clear
fi
pushd "$(pwd)" >/dev/null || exit 1

# ....Source project shell-scripts dependencies..................................................
script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
script_path_parent="$(dirname "${script_path}")"
if [[ ! $( dnp::is_lib_loaded 2>/dev/null >/dev/null )  ]]; then
  source "${script_path_parent}/../utils/import_dnp_lib.bash" || exit 1
  source "${script_path_parent}/../utils/execute_compose.bash" || exit 1
fi
if [[ -z ${SUPER_PROJECT_ROOT} ]]; then
  source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1
fi


# ====Begin========================================================================================
cd "${SUPER_PROJECT_ROOT:?err}" || exit 1

n2st::norlab_splash "${PROJECT_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
n2st::print_formated_script_header "$(basename $0) ${MSG_END_FORMAT}on device ${MSG_DIMMED_FORMAT}$(hostname -s)" "${MSG_LINE_CHAR_BUILDER_LVL2}"

n2st::set_which_architecture_and_os
n2st::print_msg "Current image architecture and os: ${IMAGE_ARCH_AND_OS:?err}"

n2st::set_is_teamcity_run_environment_variable

# ....Device specific config.......................................................................
compose_path="${DNP_ROOT:?err}/src/lib/core/docker"
the_compose_file=docker-compose.project.run.slurm.yaml

container_id=$(docker compose -f "${compose_path}/${the_compose_file}" ps --quiet --all --orphans=false)
echo
echo "Compose project project-slurm running service(s) container_id:"
echo "${container_id[*]}"
echo
echo "Stop the following running container(s):"
# shellcheck disable=SC2068
docker stop ${container_id[@]} "${remaining_args[@]}" 2>/dev/null


# ====Teardown=====================================================================================
n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
popd >/dev/null || exit 1
