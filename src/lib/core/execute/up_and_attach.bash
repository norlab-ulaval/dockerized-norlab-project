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
# =================================================================================================
EOF
)

clear
pushd "$(pwd)" >/dev/null || exit 1

# (Priority) ToDo: unit-test for flag option
# (Priority) ToDo: NMO-375 refactor: run and attach logic using 'RedLeader-research-codebase' implemention
# ToDo: refactor using Dockerized-NorLab scripts (ref task NMO-375)
# ToDo: see newly added implementation in dockerized-norlab-scripts/build_script/dn_run_a_service.bash (ref task NMO-375)


# ....Source project shell-scripts dependencies..................................................
SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]:-'.'}")"
SCRIPT_PATH_PARENT="$(dirname "${SCRIPT_PATH}")"
source "${SCRIPT_PATH_PARENT}/../utils/import_dnp_lib.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/../utils/load_super_project_config.bash" || exit 1
source "${SCRIPT_PATH_PARENT}/execute_compose.bash" || exit 1


# ToDo: move the help fct near the script/fct menu
function show_help() {
  # (NICE TO HAVE) ToDo: refactor as a n2st fct (ref NMO-583)
  echo -e "${MSG_DIMMED_FORMAT}"
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "$0 --help\n"
  # Strip shell comment char `#` and both lines
  echo -e "${DOCUMENTATION_UP_AND_ATTACH}" | sed 's/\# ====.*//' | sed 's/^\#//'
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "${MSG_END_FORMAT}"
}

# ....Set env variables (pre cli)................................................................
declare -a REMAINING_ARGS
declare -a INTERACTIVE_LOGIN
declare -a DOCKER_EXEC_CMD_AND_ARGS

THE_SERVICE=project-develop

# Note prevent double bash invocation logic (non-interactive -> interactive) when running entrypoint in up&attach
# ToDo: assess if moving to DN `dn_entrypoint.attach.bash` and `dn_entrypoint.init.bash` for all services woud be better.
INTERACTIVE_LOGIN=("-e" "BASH_ENV")

# ....cli..........................................................................................
while [ $# -gt 0 ]; do

  case $1 in
    --service)
      THE_SERVICE="${2}"
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
      REMAINING_ARGS=( "$@" )
      unset INTERACTIVE_LOGIN
      break
      ;;
    *) # Default case
      REMAINING_ARGS=("$@")
      unset INTERACTIVE_LOGIN
      break
      ;;
  esac

done

# ....Set env variables (post cli)...............................................................
# Add env var
DOCKER_EXEC_CMD_AND_ARGS=("${REMAINING_ARGS[@]:-"bash"}")

# ====Begin========================================================================================
cd "${SUPER_PROJECT_ROOT:?err}" || exit 1

n2st::norlab_splash "${PROJECT_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}"

n2st::set_which_architecture_and_os
n2st::print_msg "Current image architecture and os: $IMAGE_ARCH_AND_OS"

# ....Display and xhost............................................................................
# ToDo: assessment >> display forwarding between remote device and local [Ubuntu+MacOs]
#   check: https://www.notion.so/redleader962/e4713bb868d949b1ab93351c564f66e7?pvs=4#a5e0797fb87f4f2aa1e3c628e9492a94
#   ref task NMO-183 Fix GUI display issue
# ToDo: validate >> check jetson-container implementation
#     from https://github.com/dusty-nv/jetson-containers/blob/master/run.sh

# .................................................................................................

# ....Device specific config.......................................................................
COMPOSE_PATH=".dockerized_norlab_project/configuration"
THE_COMPOSE_FILE=""
DISPLAY_DEVICE=""
if [[ ${IMAGE_ARCH_AND_OS:?err} == 'l4t/arm64' ]] || [[ $IMAGE_ARCH_AND_OS == 'linux/x86' ]]; then

  if [[ ${IMAGE_ARCH_AND_OS:?err} == 'l4t/arm64' ]]; then
    THE_COMPOSE_FILE=docker-compose.project.run.jetson.yaml

    # copy file showing which Jetson board is running for mountinf as a volume in docker-compose
    # Source https://github.com/dusty-nv/jetson-containers/blob/master/run.sh
    cat /proc/device-tree/model >/tmp/nv_jetson_model

  elif [[ $IMAGE_ARCH_AND_OS == 'linux/x86' ]]; then
    # (Priority) inprogress: implement case >> NorLab-CI-server
    # (Priority) ToDo: implement case >> User workstation
    THE_COMPOSE_FILE=docker-compose.project.run.linux-x86.yaml
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
    # DISPLAY_DEVICE="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH"
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
  THE_COMPOSE_FILE=docker-compose.project.run.darwin.yaml

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

n2st::print_msg "Execute docker compose with ${MSG_DIMMED_FORMAT}-f ${THE_COMPOSE_FILE}${MSG_END_FORMAT}"

# ....Start docker container.......................................................................
n2st::set_is_teamcity_run_environment_variable
print_msg "IS_TEAMCITY_RUN=${IS_TEAMCITY_RUN} ${TC_VERSION}"

n2st::print_formated_script_header "$(basename $0) ${MSG_END_FORMAT}on device ${MSG_DIMMED_FORMAT}$(hostname -s)" "${MSG_LINE_CHAR_BUILDER_LVL2}"

# (CRITICAL) ToDo: see newly added container name related implementation in dockerized-norlab-scripts/build_script/dn_run_a_service.bash
if [[ $(docker compose -f "${COMPOSE_PATH}/${THE_COMPOSE_FILE}" ps --format "{{.Name}} {{.Service}} {{.State}}") == "${DN_CONTAINER_NAME} ${THE_SERVICE} running" ]]; then

  n2st::print_msg "Service ${MSG_DIMMED_FORMAT}${THE_SERVICE}${MSG_END_FORMAT} is already running"

  if [[ ${IS_TEAMCITY_RUN} == true ]]; then
    # (NICE TO HAVE) ToDo: implement >> fetch container name from an .env file
    n2st::print_msg "The container is running inside a TeamCity agent >> keep container detached"
  else
    # . . Attach to service. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    DOCKER_EXEC=("exec")
    DOCKER_EXEC+=("${INTERACTIVE_LOGIN[@]}")
    DOCKER_EXEC+=("${THE_SERVICE}")
    DOCKER_EXEC+=("/dockerized-norlab/project/${THE_SERVICE}/dn_entrypoint.attach.bash")
    DOCKER_EXEC+=("${DOCKER_EXEC_CMD_AND_ARGS[@]}")
    n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}docker compose -f ${COMPOSE_PATH}/${THE_COMPOSE_FILE} ${DOCKER_EXEC[*]}${MSG_END_FORMAT}"
    docker compose -f "${COMPOSE_PATH}/${THE_COMPOSE_FILE}" "${DOCKER_EXEC[@]}"
  fi

else

  # . . Launch service as a daemon. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  DOCKER_UP=("up" "--detach" "--wait")
  #  DOCKER_UP+=("--build")
  DOCKER_UP+=("${THE_SERVICE}")
  n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}docker compose ${DOCKER_UP[*]}${MSG_END_FORMAT}"
  docker compose -f "${COMPOSE_PATH}/${THE_COMPOSE_FILE}" "${DOCKER_UP[@]}"

  if [[ $IMAGE_ARCH_AND_OS == 'l4t/arm64' ]]; then
    # (NICE TO HAVE) ToDo: implement case fetch docker context IP address
    echo
  elif [[ $IMAGE_ARCH_AND_OS == 'darwin/arm64' ]]; then
    n2st::print_msg "Updating ssh key"
    bash -c "ssh-keygen -R [localhost]:2222"
  fi

  if [[ ${IS_TEAMCITY_RUN} == true ]]; then
    # (NICE TO HAVE) ToDo: implement >> fetch container name from an .env file
    n2st::print_msg "The container is running inside a TeamCity agent >> keep container detached"
  else
    # . . Attach to service. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    DOCKER_EXEC=("exec")
    DOCKER_EXEC+=("${INTERACTIVE_LOGIN[@]}")
    DOCKER_EXEC+=("${THE_SERVICE}")
    # Note: The init entrypoint is executed here on purpose, not at docker compose up.
    DOCKER_EXEC+=("/dockerized-norlab/project/${THE_SERVICE}/dn_entrypoint.init.bash")
    DOCKER_EXEC+=("${DOCKER_EXEC_CMD_AND_ARGS[@]}")
    n2st::print_msg "Execute ${MSG_DIMMED_FORMAT}docker compose -f ${COMPOSE_PATH}/${THE_COMPOSE_FILE} ${DOCKER_EXEC[*]}${MSG_END_FORMAT}"
    docker compose -f "${COMPOSE_PATH}/${THE_COMPOSE_FILE}" "${DOCKER_EXEC[@]}"
  fi
fi

# ====Teardown=====================================================================================
n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL2}"
popd >/dev/null || exit 1
