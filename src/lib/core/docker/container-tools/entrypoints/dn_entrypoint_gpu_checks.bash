#!/bin/bash

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

host_gpu_architecture = '${host_gpu_architecture}'
torch_compiled_arch_support = torch.cuda.get_arch_list() if torch.cuda.is_available() else []

if host_gpu_architecture in torch_compiled_arch_support:
    print('true')
else:
    print('false')

" 2>/dev/null) || return 1

    echo "${is_host_gpu_to_container_torch_compatible}"

  else
    # Torch is not available in container
    echo "no-torch"
  fi
  return 0
}

dna::test_container_torch_supported_architecture "$@"
exit $?
