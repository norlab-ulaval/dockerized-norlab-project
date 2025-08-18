#!/bin/bash
# =================================================================================================
# CUDA and GPU compatibility utilities for NVIDIA Docker runtime configuration.
#
# Provides functions to detect host NVIDIA GPU architecture, validate CUDA development tools,
# test PyTorch compatibility between host GPU and container, and configure Docker runtime
# settings based on target platform architecture.
#
# Usage:
#   $ source cuda_tools.bash
#
# Globals:
#   Read/Write NVIDIA_VISIBLE_DEVICES - Controls GPU visibility to containers
#   Read/Write NVIDIA_DRIVER_CAPABILITIES - Specifies NVIDIA driver capabilities
#   Read/Write DN_DOCKER_RUNTIME - Sets Docker runtime (nvidia or runc)
#   Write DN_HOST_GPU_ARCHITECTURE - Host GPU architecture detection result
#   Read DNA_DEBUG - Enable debug output when set to true
#
# =================================================================================================


# =================================================================================================
# Extracts NVIDIA GPU architecture information from host system.
#
# Usage:
#   $ gpu_arch=$(dna::fetch_host_nvidia_gpu_architecture "image_arch_and_os")
#   $ echo "Host GPU architecture: $gpu_arch"
#
# Positional argument:
#   image_arch_and_os - Target image architecture and OS (e.g., 'darwin/arm64', 'l4t/arm64', 'linux/x86')
#
# Outputs:
#   Writes GPU architecture string to stdout in format "sm_XX" or "NO-NVIDIA-GPU-SUPPORT" if either
#   "nvidia-container-cli" or "nvidia-smi" + "nvcc" are not  installed on host.
# =================================================================================================
function dna::fetch_host_nvidia_gpu_architecture() {
  local image_arch_and_os=${1:?err}

  if [[ $image_arch_and_os == 'linux/x86' ]]; then
    if nvidia-container-cli -V  &> /dev/null ; then
      # Try to get architecture from nvidia-container-cli
      local arch_output
      arch_output=$(nvidia-container-cli info 2>/dev/null | grep "Architecture:" | awk '{print $2}' | tr -d '.')

      if [[ -n "$arch_output" ]]; then
        #export DN_HOST_GPU_ARCHITECTURE="sm_${arch_output}"
        echo "sm_${arch_output}"
        return 0
      fi
    fi
  elif [[ $image_arch_and_os == 'l4t/arm64' ]]; then
    # Fallback for Jetson devices or when nvidia-container-cli fails
    if dna::check_nvidia_cuda_support &> /dev/null; then
      # Try to get architecture from deviceQuery or nvidia-smi
      local jetson_arch
      jetson_arch=$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d '.')

      if [[ -n "$jetson_arch" ]]; then
        #export DN_HOST_GPU_ARCHITECTURE="sm_${jetson_arch}"
        echo "sm_${jetson_arch}"
        return 0
      fi
    fi
  else
    # Assume no gpu support since neither method worked
    #export DN_HOST_GPU_ARCHITECTURE="NO-NVIDIA-GPU-SUPPORT"
    echo "NO-NVIDIA-GPU-SUPPORT"
    return 0
  fi
}

# =================================================================================================
# Checks if NVIDIA CUDA development tools are available on the system.
#
# Usage:
#   $ if dna::check_nvidia_cuda_support; then
#   $   echo "CUDA is available"
#   $ fi
#
# Returns:
#   0 if cuda is compatible, 1 oterwise
# =================================================================================================
function dna::check_nvidia_cuda_support() {
  # Note: nvcc requires an interactive shell, otherwise it output nothing
  if [[ $(bash -i -c "nvcc -V && exit" 2>/dev/null | grep 'nvcc: NVIDIA (R) Cuda compiler driver') == "nvcc: NVIDIA (R) Cuda compiler driver" ]]; then
    return 0
  else
    return 1
  fi
}

# =================================================================================================
# Tests if host GPU architecture is compatible with container's PyTorch version.
#
# Verifies compatibility by running a PyTorch architecture test inside the
# specified container using the host's GPU architecture as input. This ensures
# the container's PyTorch installation can properly utilize the host's GPU.
#
# Usage example:
#   $ dna::test_host_gpu_to_container_torch_compatibility /path/to/compose docker-compose.yml gpu-service sm_75
#
# Arguments:
#   compose_path - Path to directory containing compose file
#   the_compose_file - Name of the compose file
#   the_service - Docker compose service name to test
#   host_gpu_arch - GPU architecture string from host system
#
# Outputs:
#   Writes to stdout: "true" if compatible, "false" if incompatible, "no-torch" if PyTorch unavailable
#   Writes error messages to stderr via n2st::print_msg_error
#
# Returns:
#   0 if GPU is compatible, 1 if incompatible or test fails
# =================================================================================================
function dna::test_host_gpu_to_container_torch_compatibility() {
  # ....Setup......................................................................................
  local compose_path="${1:?err}"
  local the_compose_file="${2:?err}"
  local the_service="${3:?err}"
  local host_gpu_arch="${4:?err}"
  local is_host_gpu_to_container_torch_compatible

  # Set runtime value for docker compose run
  export NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-all}"
  export NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:-all}"
  export DN_DOCKER_RUNTIME="${DN_DOCKER_RUNTIME:-nvidia}"

  # ....Begin......................................................................................
  declare -a docker_flags=()
  if [[ ${DNA_DEBUG} == true ]]; then
    #docker_flags+=("--debug")
    docker_flags+=("--log-level" "debug")
  fi
  declare -a docker_cmd=("run")
  docker_cmd+=("--rm")
#  docker_cmd+=("--interactive")
#  docker_cmd+=("--no-TTY")
#  docker_cmd+=("--no-deps")
#  docker_cmd+=("--remove-orphans")
#  docker_cmd+=("--cap-drop")
#  docker_cmd+=("--name" "${DN_CONTAINER_NAME:?err}-gpu-test-${BASHPID:-$$}")
  docker_cmd+=("--entrypoint" "/bin/bash -c")
  docker_cmd+=("${the_service}")
  docker_cmd+=("/dna-lib-container-tools/project_entrypoints/dn_entrypoint_gpu_checks.bash '${host_gpu_arch}'")

  is_host_gpu_to_container_torch_compatible=$(docker "${docker_flags[@]}" compose -f "${compose_path}/${the_compose_file}" "${docker_cmd[@]}")
  local exit_code=$?

  # ....Teardown...................................................................................
  if [[ ${exit_code} -ne 0 ]]; then
    n2st::print_msg_error "dna::test_host_gpu_to_container_torch_compatibility exited with error!"
    return 1
  fi

  echo "${is_host_gpu_to_container_torch_compatible}"
  return 0
}

# =================================================================================================
# Configures GPU capabilities and Docker runtime based on target architecture.
#
# This function determines the appropriate NVIDIA GPU settings and Docker runtime
# configuration based on the target image architecture and host GPU compatibility.
# It handles different scenarios including ARM64 Darwin (no GPU), L4T ARM64, and
# Linux x86 platforms with varying GPU support levels.
#
# Usage:
#   $ dna::configure_gpu_capabilities "linux/x86" /path/to/compose docker-compose.yml gpu-service
#   $ dna::configure_gpu_capabilities "darwin/arm64" /path/to/compose docker-compose.yml gpu-service
#
# Arguments:
#   image_arch_and_os - Target image architecture and OS (e.g., 'darwin/arm64', 'l4t/arm64', 'linux/x86')
#   compose_path - Path to directory containing compose file
#   the_compose_file - Name of the compose file
#   the_service - Docker compose service name to test
#
# Globals:
#   Read/Write NVIDIA_VISIBLE_DEVICES - Controls which GPUs are visible to container
#   Read/Write NVIDIA_DRIVER_CAPABILITIES - Specifies NVIDIA driver capabilities
#   Read/Write DN_DOCKER_RUNTIME - Sets the Docker runtime (nvidia or runc)
#
# Outputs:
#   Writes configuration messages to stdout
#   Writes warning messages to stdout via n2st::print_msg_warning
#
# Returns:
#   0 - Success
#   1 - Failure (via exit in subprocess)
# =================================================================================================
function dna::configure_gpu_capabilities() {
  local image_arch_and_os=${1:?err}
  local compose_path="${2:?err}"
  local the_compose_file="${3:?err}"
  local the_service="${4:?err}"

  # ....Debug: pre.................................................................................
  if [[ ${DNA_DEBUG} == true ]]; then
    n2st::print_msg "Current container on host..."
    docker container ls -a
    echo
    n2st::print_msg "pre-dna::configure_gpu_capabilities related environment variable...
    NVIDIA_VISIBLE_DEVICES: $NVIDIA_VISIBLE_DEVICES
    NVIDIA_DRIVER_CAPABILITIES: $NVIDIA_DRIVER_CAPABILITIES
    DN_DOCKER_RUNTIME: $DN_DOCKER_RUNTIME
    "
  fi

  # ....Begin......................................................................................
  if [[ $NVIDIA_VISIBLE_DEVICES == void ]] || [[ $DN_DOCKER_RUNTIME == runc ]]; then
    n2st::print_msg "No nvidia gpu support expected by user"
    NVIDIA_VISIBLE_DEVICES=void
    NVIDIA_DRIVER_CAPABILITIES=""
    DN_DOCKER_RUNTIME=runc
    DN_HOST_GPU_ARCHITECTURE=$(dna::fetch_host_nvidia_gpu_architecture "$image_arch_and_os")
  elif [[ ${image_arch_and_os:?err} == 'darwin/arm64' ]]; then
    n2st::print_msg "No MacOs gpu support in container:"
    NVIDIA_VISIBLE_DEVICES=void
    NVIDIA_DRIVER_CAPABILITIES=""
    DN_DOCKER_RUNTIME=runc
    DN_HOST_GPU_ARCHITECTURE="apple_silicon"
  elif [[ $image_arch_and_os == 'l4t/arm64' ]] || [[ $image_arch_and_os == 'linux/x86' ]]; then
    DN_HOST_GPU_ARCHITECTURE=$(dna::fetch_host_nvidia_gpu_architecture "$image_arch_and_os")
    #test -n "${DN_HOST_GPU_ARCHITECTURE:?'Env variable need to be set and non-empty.'}"

    if [[ ${DNA_DEBUG} == true ]]; then
      n2st::print_msg "dna::fetch_host_nvidia_gpu_architecture -> $DN_HOST_GPU_ARCHITECTURE"
    fi

    if [[ "${DN_HOST_GPU_ARCHITECTURE:?err}" != "NO-NVIDIA-GPU-SUPPORT" ]]; then
      # ...........................................................................................
      # Cases:
      #   1. host support cuda -> user nvidia env var passed to container
      #   2. torch installed in container -> validate that its compatible with host device
      # ...........................................................................................

      if [[ ${DNA_DEBUG} == true ]]; then
        echo "run dna::test_host_gpu_to_container_torch_compatibility..."
      fi
      is_host_gpu_to_container_torch_compatible=$( dna::test_host_gpu_to_container_torch_compatibility "${compose_path}" "${the_compose_file}" "${the_service}" "${DN_HOST_GPU_ARCHITECTURE:?err}" )
      local torch_test_exit_code=$?
      if [[ ${DNA_DEBUG} == true ]]; then
        n2st::print_msg "is_host_gpu_to_container_torch_compatible -> $is_host_gpu_to_container_torch_compatible"
      fi

      if [[ ${torch_test_exit_code} -ne 0 ]]; then
        return 1
      elif [[ "${is_host_gpu_to_container_torch_compatible}" =~ ^(true|no-torch)$ ]]; then
        n2st::print_msg "Nvidia gpu capabilities enable:"
        NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-all}"
        NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:-all}"
        DN_DOCKER_RUNTIME=nvidia
      else
        n2st::print_msg_warning "Container has torch installed and expected gpu support but host device is not compatible!"
        echo "Disabling nvidia docker runtime."
        NVIDIA_VISIBLE_DEVICES=void
        NVIDIA_DRIVER_CAPABILITIES=""
        DN_DOCKER_RUNTIME=runc
      fi
    elif [[ "${host_gpu_arch}" == "NO-NVIDIA-GPU-SUPPORT" ]]; then
      n2st::print_msg "No nvidia gpu support on host."
      NVIDIA_VISIBLE_DEVICES=void
      NVIDIA_DRIVER_CAPABILITIES=""
      DN_DOCKER_RUNTIME=runc
    fi
  fi

  # ....Debug: post................................................................................
  if [[ ${DNA_DEBUG} == true ]]; then
    n2st::print_msg "Current container on host..."
    docker container ls -a
    echo
  fi

  # ....User feedback..............................................................................
  # Note: Section title is set in each 'if/the/else' clause
  if [[ ${image_arch_and_os:?err} != 'darwin/arm64' ]]; then
    echo "  NVIDIA_VISIBLE_DEVICES: ${NVIDIA_VISIBLE_DEVICES}"
    echo "  NVIDIA_DRIVER_CAPABILITIES: ${NVIDIA_DRIVER_CAPABILITIES}"
  fi
  echo "  DN_DOCKER_RUNTIME: ${DN_DOCKER_RUNTIME}"
  echo "  DN_HOST_GPU_ARCHITECTURE: ${DN_HOST_GPU_ARCHITECTURE}"

  # ....Teardown...................................................................................
  export NVIDIA_VISIBLE_DEVICES
  export NVIDIA_DRIVER_CAPABILITIES
  export DN_DOCKER_RUNTIME
  export DN_HOST_GPU_ARCHITECTURE
  return 0
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dna_error_prefix="\033[1;31m[dna error]\033[0m"
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${dna_error_prefix} This script must be sourced i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # Check if N2ST is loaded
  test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
fi
