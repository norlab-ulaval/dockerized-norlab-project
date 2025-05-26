#!/bin/bash
#
# Usage
#   $ cd .dockerized_norlab_project/execute
#   $ bash cadence/run_cadence.ci_test.bash
#
bash build.ci_tests.bash && bash run.ci_tests.bash
