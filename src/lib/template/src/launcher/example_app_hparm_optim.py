# coding=utf-8
import random
from typing import Tuple

import omegaconf
import hydra
import os

from example_app import run_pytorch_check


@hydra.main(
    config_path="configs", config_name="example_app_hparm_optim", version_base=None
)
def hyperparam_opt_pipeline(cfg: omegaconf.DictConfig) -> Tuple[float, ...]:
    # .... Optuna related .........................................................................
    storage_root = f"{cfg.project_experiment_root_path}/optuna_storage"
    if not os.path.exists(storage_root):
        os.makedirs(storage_root)

    # .... Overriding cfg for multirun ............................................................
    omegaconf.OmegaConf.update(cfg, "simulation_mode.rendering", "headless_fast", merge=False)

    dna_sjob_name = os.getenv("DNA_SJOB_NAME")
    if dna_sjob_name:
        assert dna_sjob_name == cfg.hparam_optimizer.dna_sjob_name, (
            f"Missconfiguration: slurm_job.*.bash "
            f"DNA_SJOB_NAME={dna_sjob_name} != cfg.hparam_optimizer.dna_sjob_name={cfg.hparam_optimizer.dna_sjob_name}"
        )

    # .... Execute multirun .......................................................................
    run_pytorch_check(cfg)

    return random.random()


if __name__ == "__main__":
    hyperparam_opt_pipeline()
