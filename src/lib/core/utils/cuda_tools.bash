#!/bin/bash


# =================================================================================================
# Extracts NVIDIA GPU architecture information from host system.
#
# Usage:
#   $ gpu_arch=$(dna::fetch_host_gpu_architecture)
#   $ echo "Host GPU architecture: $gpu_arch"
#
# Outputs:
#   Writes GPU architecture string to stdout in format "sm_XX" or "NO-NVIDIA-GPU-SUPPORT"
#   if "nvidia-container-cli" is not installed on host i.e., no gpu to docker support on host.
# =================================================================================================
function dna::fetch_host_gpu_architecture() {
  if nvidia-container-cli -V  &> /dev/null ; then
    # Try to get architecture from nvidia-container-cli
    local arch_output
    arch_output=$(nvidia-container-cli info 2>/dev/null | grep "Architecture:" | awk '{print $2}' | tr -d '.')

    if [[ -n "$arch_output" ]]; then
      echo "sm_${arch_output}"
      return 0
    fi
  fi

  # Fallback for Jetson devices or when nvidia-container-cli fails
  if command -v nvcc &> /dev/null; then
    # Try to get architecture from deviceQuery or nvidia-smi
    local jetson_arch
    jetson_arch=$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d '.')

    if [[ -n "$jetson_arch" ]]; then
      echo "sm_${jetson_arch}"
      return 0
    fi
  fi

  # Assume no gpu support since neither method worked
  echo "NO-NVIDIA-GPU-SUPPORT"
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
  if [[ $(nvcc -V 2>/dev/null | grep 'nvcc: NVIDIA (R) Cuda compiler driver') == "nvcc: NVIDIA (R) Cuda compiler driver" ]]; then
    return 0
  else
    return 1
  fi
}

# =================================================================================================
# Tests if host GPU architecture is compatible with container's installed PyTorch version.
#
#   1. Check if PyTorch is available
#   2. Checks if the host GPU architecture is supported by the PyTorch CUDA compilation.
#
# Usage example:
#   $ dna::test_container_torch_supported_architecture "sm_75"
#
# Arguments:
#   host_gpu_architecture - GPU architecture identifier (e.g., "sm_75")
#
# Outputs:
#   Writes to stdout: "true" if compatible, "false" if incompatible, "no-torch" if PyTorch unavailable
# =================================================================================================
function dna::test_container_torch_supported_architecture() {
  local host_gpu_architecture="${1:?err}"
  local is_host_gpu_to_container_torch_compatible

  if pip -qq show torch; then
    is_host_gpu_to_container_torch_compatible=$(python3 -c "
import torch

host_gpu_architecture = ${host_gpu_architecture}
torch_compiled_arch_support = (
  torch.cuda.get_arch_list() if torch.cuda.is_available() else 'None'
  )

if host_gpu_architecture in torch_compiled_arch_support:
  print('true')
else
  print('false')

" 2>/dev/null) || return 1

    echo "${is_host_gpu_to_container_torch_compatible}"

  else
    # Torch is not available in container
    echo "no-torch"
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

  # ....Begin......................................................................................
  declare -a docker_cmd=("run")
  docker_cmd+=("${the_service}")
#  docker_cmd+=("/usr/local/bin/bash-dn-buildtime" "-c" "")
  docker_cmd+=("/usr/local/bin/bash-dn-non-interactive-ros2" "-c" "$(declare -f dna::test_container_torch_supported_architecture); dna::test_container_torch_supported_architecture ${host_gpu_arch}")
  n2st::print_msg "Check host vs container gpu compatibility ${MSG_DIMMED_FORMAT}docker compose ${docker_cmd[*]}${MSG_END_FORMAT}"
  is_host_gpu_to_container_torch_compatible=$(docker compose -f "${compose_path}/${the_compose_file}" "${docker_cmd[@]}")
  local exit_code=$?

  # ....Teardown...................................................................................
  if [[ ${exit_code} -eq 0 ]]; then
    echo "${is_host_gpu_to_container_torch_compatible}"
    return 0
  else
    n2st::print_msg_error "dna::test_host_gpu_to_container_torch_compatibility exited with error!"
    return 1
  fi
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
#   Write DN_DOCKER_RUNTIME - Sets the Docker runtime (nvidia or runc)
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

  DN_DOCKER_RUNTIME=runc

  # ....Begin......................................................................................
  if [[ $NVIDIA_VISIBLE_DEVICES == void ]]; then
    # No nvidia gpu support expected by user
    NVIDIA_DRIVER_CAPABILITIES=""
  elif [[ ${image_arch_and_os:?err} == 'darwin/arm64' ]]; then
    # No nvidia gpu support on macOs
    n2st::print_msg_warning "Host computer does not support nvidia gpu."
    NVIDIA_VISIBLE_DEVICES=void
    NVIDIA_DRIVER_CAPABILITIES=""
  elif [[ $image_arch_and_os == 'l4t/arm64' ]] || [[ $image_arch_and_os == 'linux/x86' ]]; then

    host_gpu_arch=$(dna::fetch_host_gpu_architecture)

    if [[ "${host_gpu_arch}" == "NO-NVIDIA-GPU-SUPPORT" ]]; then
      n2st::print_msg "No nvidia gpu support on host"
      NVIDIA_VISIBLE_DEVICES=void
      NVIDIA_DRIVER_CAPABILITIES=""
    else
      # Cases:
      #   1. torch installed in container -> validate that its compatible with host device
      #   2. host support cuda -> user nvidia env var passed to container
      is_host_gpu_to_container_torch_compatible=$(
        dna::test_host_gpu_to_container_torch_compatibility "${compose_path}" \
           "${the_compose_file}" "${the_service}" "${host_gpu_arch}"
        )
      local torch_test_exit_code=$?
      
      if [[ ${torch_test_exit_code} -ne 0 ]]; then
        return 1
      fi

      if [[ $NVIDIA_VISIBLE_DEVICES != void ]] && [[ "${is_host_gpu_to_container_torch_compatible}" =~ ^(true|no-torch)$ ]]; then
        n2st::print_msg "Nvidia gpu support on host enable:"
        NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-all}"
        NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:-all}"
        DN_DOCKER_RUNTIME=nvidia
        echo "  NVIDIA_VISIBLE_DEVICES: ${NVIDIA_VISIBLE_DEVICES}"
        echo "  NVIDIA_DRIVER_CAPABILITIES: ${NVIDIA_DRIVER_CAPABILITIES}"
      else
        n2st::print_msg_warning "Container has torch installed and expected gpu support but host device is not compatible!"
        echo "Disabling nvidia docker runtime."
        NVIDIA_VISIBLE_DEVICES=void
        NVIDIA_DRIVER_CAPABILITIES=""
      fi
    fi
  fi

  # ....Teardown...................................................................................
  export NVIDIA_VISIBLE_DEVICES
  export NVIDIA_DRIVER_CAPABILITIES
  export DN_DOCKER_RUNTIME
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
