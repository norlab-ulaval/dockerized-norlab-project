#!/bin/bash

# ....path resolution logic........................................................................
SUPER_PROJECT_ROOT=$(git rev-parse --show-toplevel)
cd "${SUPER_PROJECT_ROOT}"/.dockerized_norlab_project/execute

# ====begin========================================================================================
bash build.develop.bash
EXIT_CODE=$?
if [[ ${EXIT_CODE} != 0 ]]; then
  exit $EXIT_CODE
fi

bash up_and_attach.bash --service project-develop -- tree -L 1
EXIT_CODE=$?
if [[ ${EXIT_CODE} != 0 ]]; then
  exit $EXIT_CODE
fi

bash down.bash
EXIT_CODE=$?
if [[ ${EXIT_CODE} != 0 ]]; then
  exit $EXIT_CODE
fi

