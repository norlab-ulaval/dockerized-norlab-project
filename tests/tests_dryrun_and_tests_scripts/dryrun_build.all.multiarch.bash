#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_dotenv.bash" || exit 1
cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# ====begin========================================================================================
bash "${DNP_LIB_EXEC_PATH:?err}"/build.all.multiarch.bash --no-force-push-project-core -- --dry-run
exit $?
