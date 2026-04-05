import os
import sys

print("APPTAINER_TEST_OK")
print("DN_PROJECT_PATH=" + os.environ.get("DN_PROJECT_PATH", "UNSET"))
print("DN_PROJECT_USER=" + os.environ.get("DN_PROJECT_USER", "UNSET"))
print("IS_SLURM_RUN=" + os.environ.get("IS_SLURM_RUN", "UNSET"))
print("DNA_RUNTIME=" + os.environ.get("DNA_RUNTIME", "UNSET"))
print("SLURM_JOB_ID=" + os.environ.get("SLURM_JOB_ID", "UNSET"))
sys.exit(0)
