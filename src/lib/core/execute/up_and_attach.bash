#!/bin/bash
DOCUMENTATION_UP_AND_ATTACH=$( cat <<'EOF'
# =================================================================================================
# Convenient script for spinning a specific service from 'docker-compose.project.run.<DEVICE>.yaml'
# Under the hood, start the service in detach mode and attach to it so that you can close
# the terminal and it will keep running in the background.
#
# Usage:
#   $ bash up_and_attach.bash [--service <theService>]  [--] [<command&arguments>]
#
# Arguments:
#   --service                The service to attach once up (Default: project-develop)
#   -h | --help
#
# Positional argument:
#   <command&arguments>      Any command to be executed inside the docker container (default: bash)
#
# Globals:
#   read DNP_ROOT
#   read SUPER_PROJECT_ROOT
#   read DN_CONTAINER_NAME
#   read/write DISPLAY (optional)
#
# =================================================================================================
EOF
)
# (Priority) ToDo: unit-test for flag option
# (Priority) ToDo: NMO-375 refactor: run and attach logic using 'RedLeader-research-codebase' implemention
# ToDo: refactor using Dockerized-NorLab scripts (ref task NMO-375)
# ToDo: see newly added implementation in dockerized-norlab-scripts/build_script/dn_run_a_service.bash (ref task NMO-375)

function show_help() {
  # (NICE TO HAVE) ToDo: refactor as a n2st fct (ref NMO-583)
  echo -e "${MSG_DIMMED_FORMAT}"
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "$0 --help"
  # Strip shell comment char `#` and both lines
  echo -e "${DOCUMENTATION_UP_AND_ATTACH}" | sed '/\# ====.*/d' | sed 's/^\# //' | sed 's/^\#//'
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "${MSG_END_FORMAT}"
}

function dnp::up_and_attach() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  # ....Set env variables (pre cli)................................................................
  declare -a remaining_args
  declare -a interactive_login
  local the_service=project-develop

  # Note prevent double bash invocation logic (non-interactive -> interactive) when running entrypoint in up&attach
  # ToDo: assess if moving to DN `dn_entrypoint.attach.bash` and `dn_entrypoint.init.bash` for all services woud be better.
  interactive_login=("-e" "BASH_ENV")

  # ....cli........................................................................................
  while [ $# -gt 0 ]; do

    case $1 in
      --service)
        the_service="${2}"
        shift # Remove argument (--service)
        shift # Remove argument value
        ;;
      -h | --help)
        clear
        show_help
        exit
        ;;
      --) # no more option
        shift
        remaining_args=( "$@" )
        unset interactive_login
        break
        ;;
      *) # Default case
        remaining_args=("$@")
        unset interactive_login
        break
        ;;
    esac

  done

  # ....Set env variables (post cli)...............................................................
  declare -a docker_exec_cmd_and_args=("${remaining_args[@]:-"bash"}")
  local compose_path="${DNP_ROOT:?err}/src/lib/core/docker"
  local the_compose_file=""
  local display_device=""
  local up_exit_code
  local exec_exit_code

  # ====Begin======================================================================================
  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1

  # ....Display and xhost..........................................................................
  # ToDo: assessment >> display forwarding between remote device and local [Ubuntu+MacOs]
  #   check: https://www.notion.so/redleader962/e4713bb868d949b1ab93351c564f66e7?pvs=4#a5e0797fb87f4f2aa1e3c628e9492a94
  #   ref task NMO-183 Fix GUI display issue
  # ToDo: validate >> check jetson-container implementation
  #     from https://github.com/dusty-nv/jetson-containers/blob/master/run.sh

  # ....Device specific config.....................................................................


  n2st::set_which_architecture_and_os
  n2st::print_msg "Current image architecture and os: ${IMAGE_ARCH_AND_OS:?err}"
  if [[ ${IMAGE_ARCH_AND_OS:?err} == 'l4t/arm64' ]] || [[ $IMAGE_ARCH_AND_OS == 'linux/x86' ]]; then

    if [[ ${IMAGE_ARCH_AND_OS:?err} == 'l4t/arm64' ]]; then
      the_compose_file=docker-compose.project.run.jetson.yaml

      # copy file showing which Jetson board is running for mountinf as a volume in docker-compose
      # Source https://github.com/dusty-nv/jetson-containers/blob/master/run.sh
      cat /proc/device-tree/model >/tmp/nv_jetson_model

    elif [[ $IMAGE_ARCH_AND_OS == 'linux/x86' ]]; then
      # (Priority) inprogress: implement case >> NorLab-CI-server
      # (Priority) ToDo: implement case >> User workstation
      the_compose_file=docker-compose.project.run.linux-x86.yaml
    fi

    if [ -n "$DISPLAY" ]; then
      # Credit: dusty-nv
      # Source: https://github.com/dusty-nv/jetson-containers/blob/master/run.sh

      # give docker user X11 permissions
      # Note:
      # - 'root' is the username here (the one in the docker container)
      sudo xhost +si:localuser:root
      #    sudo xhost +si:localuser:"${DN_PROJECT_USER:?err}"

      # enable SSH X11 forwarding inside container (https://stackoverflow.com/q/48235040)
      # Note: XAUTH is also hardcoded in the docker compose file
      XAUTH=/tmp/.docker.xauth
      #    touch $XAUTH
      #    # Create the '.Xauthority' if not not using X11 forwarding remotely
      #    touch ~/.Xauthority
      xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
      sudo chmod 777 $XAUTH

      # Note: can't pass those argument to "docker compose up" only to "docker compose run"
      # display_device="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH"
    else
      # Quick ref notes:
      #   - To open the display: $ xset -display :0 dpms force on
      #   - To fetch info on display: $ xset -display :0 q
      #   - To fetch info on display (alt): $ xrandr -d :0
      #   - To check connected device: $ xrandr -q
      #   - Check/set configuration via $ nvidia-xconfig
      #   - NVIDIA
      #     - https://gist.github.com/shehzan10/8d36c908af216573a1f0#remote-opengl-setup-without-x
      #     - https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/SD/WindowingSystems/XWindowSystem.html

      # NO display set in current bash session, set display for OpenGl rendering in headless mode
      export DISPLAY=:0

      # give docker user X11 permissions
      xhost +si:localuser:root
      #    xhost +si:localuser:"${DN_PROJECT_USER:?err}"
    fi

  elif [[ $IMAGE_ARCH_AND_OS == 'darwin/arm64' ]]; then
    the_compose_file=docker-compose.project.run.darwin.yaml

    # Enable IGLX for X11 forwarding with OpenGL support
    # To test X11 forwarding with OpenGL, run in the container
    #     $ glxgears
    # The animated gears should be displayed on your local computer in macOS XQuartz app.
    defaults write org.xquartz.X11 enable_iglx -bool YES
    defaults read org.xquartz.X11 enable_iglx

    xhost +localhost
    export DN_DISPLAY=host.docker.internal:0

  else
    n2st::print_msg_error_and_exit "Support for current host not implemented yet!"
  fi

  n2st::print_msg "Execute docker compose with ${MSG_DIMMED_FORMAT}-f ${the_compose_file}${MSG_END_FORMAT}"

  # ....Start docker container.....................................................................
  n2st::set_is_teamcity_run_environment_variable
  print_msg "IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN:?err} ${TC_VERSION}"

  n2st::print_msg "Starting container on device ${MSG_DIMMED_FORMAT}$(hostname -s)${MSG_END_FORMAT}"
  # n2st::print_formated_script_header "$(basename $0) ${MSG_END_FORMAT}on device ${MSG_DIMMED_FORMAT}$(hostname -s)" "${MSG_LINE_CHAR_BUILDER_LVL2}"

  # (CRITICAL) ToDo: see newly added container name related implementation in dockerized-norlab-scripts/build_script/dn_run_a_service.bash
  if [[ $(docker compose -f "${compose_path}/${the_compose_file}" ps --format "{{.Name}} {{.Service}} {{.State}}") == "${DN_CONTAINER_NAME:?err} ${the_service} running" ]]; then

    n2st::print_msg "Service ${MSG_DIMMED_FORMAT}${the_service}${MSG_END_FORMAT} is already running"
    up_exit_code=0

    if [[ ${IS_TEAMCITY_RUN} == true ]]; then
      # (NICE TO HAVE) ToDo: implement >> fetch container name from an .env file
      n2st::print_msg "The container is running inside a TeamCity agent >> keep container detached"
    else
      # . . Attach to service. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      declare -a docker_exec_no_up=("exec")
      docker_exec_no_up+=("${interactive_login[@]}")
      docker_exec_no_up+=("${the_service}")
      docker_exec_no_up+=("/dockerized-norlab/project/${the_service}/dn_entrypoint.attach.bash")
      docker_exec_no_up+=("${docker_exec_cmd_and_args[@]}")
      n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}docker compose -f ${compose_path}/${the_compose_file} ${docker_exec_no_up[*]}${MSG_END_FORMAT}"
      docker compose -f "${compose_path}/${the_compose_file}" "${docker_exec_no_up[@]}"
      exec_exit_code=$?
    fi

  else

    # . . Launch service as a daemon. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    declare -a docker_up=("up" "--detach" "--wait")
    #  docker_up+=("--build")
    docker_up+=("${the_service}")
    n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}docker compose ${docker_up[*]}${MSG_END_FORMAT}"
    docker compose -f "${compose_path}/${the_compose_file}" "${docker_up[@]}"
    up_exit_code=$?

    if [[ $IMAGE_ARCH_AND_OS == 'l4t/arm64' ]]; then
      # (NICE TO HAVE) ToDo: implement case fetch docker context IP address
      :
    elif [[ $IMAGE_ARCH_AND_OS == 'darwin/arm64' ]]; then
      n2st::print_msg "Updating ssh key"
      bash -c "ssh-keygen -R [localhost]:2222"
    fi

    if [[ ${IS_TEAMCITY_RUN} == true ]]; then
      # (NICE TO HAVE) ToDo: implement >> fetch container name from an .env file
      n2st::print_msg "The container is running inside a TeamCity agent >> keep container detached"
    else
      # . . Attach to service. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      declare -a docker_exec=("exec")
      docker_exec+=("${interactive_login[@]}")
      docker_exec+=("${the_service}")
      # Note: The init entrypoint is executed here on purpose, not at docker compose up.
      docker_exec+=("/dockerized-norlab/project/${the_service}/dn_entrypoint.init.bash")
      docker_exec+=("${docker_exec_cmd_and_args[@]}")
      n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}docker compose -f ${compose_path}/${the_compose_file} ${docker_exec[*]}${MSG_END_FORMAT}"
      docker compose -f "${compose_path}/${the_compose_file}" "${docker_exec[@]}"
      exec_exit_code=$?
    fi
  fi

  # ....Teardown...................................................................................
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return $((up_exit_code + exec_exit_code))
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  script_path_parent="$(dirname "${script_path}")"
  if [[ -z $( declare -F dnp::import_lib_and_dependencies ) ]]; then
    source "${script_path_parent}/../utils/import_dnp_lib.bash" || exit 1
    source "${script_path_parent}/../utils/execute_compose.bash" || exit 1
  fi
  if [[ -z ${SUPER_PROJECT_ROOT} ]]; then
    source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1
  fi

  # ....Execute....................................................................................
  if [[ "${DNP_CLEAR_CONSOLE_ACTIVATED}" == "true" ]]; then
    clear
  fi
  n2st::norlab_splash "${DNP_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dnp::up_and_attach "$@"
  fct_exit_code=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${fct_exit_code}"
else
  # This script is being sourced, ie: __name__="__source__"

  # ....Pre-condition..............................................................................
  dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
  test -n "$( declare -F dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
  test -n "$( declare -F n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
  test -n "${SUPER_PROJECT_ROOT}" || { echo -e "${dnp_error_prefix} The super project DNP configuration is not loaded!" ; exit 1 ; }
fi

