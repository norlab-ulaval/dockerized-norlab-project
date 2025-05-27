#!/bin/bash
#
# Usage
#   $ cd src/lib/core/execute
#   $ bash cadence/run_cadence.ci_test.bash
#
bash build.ci_tests.bash && bash run.ci_tests.bash
