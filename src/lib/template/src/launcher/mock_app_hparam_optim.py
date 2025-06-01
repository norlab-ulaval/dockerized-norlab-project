# coding=utf-8
import random
from typing import Tuple

import omegaconf
import hydra
import os

from mock_app import run_pytorch_check


@hydra.main(
    config_path="configs", config_name="example_hparm_optim_exp_config", version_base=None
)
def hyperparam_opt_pipeline(cfg: omegaconf.DictConfig) -> Tuple[float, ...]:
    # .... Optuna related .........................................................................
    storage_root = f"{cfg.project_experiment_root_path}/optuna_storage"
    if not os.path.exists(storage_root):
        os.makedirs(storage_root)

    # .... Overriding cfg for multirun ............................................................
    omegaconf.OmegaConf.update(cfg, "simulation_mode.rendering", "headless_fast", merge=False)

    sjob_id = os.getenv("SJOB_ID")
    if sjob_id:
        assert sjob_id == cfg.hparam_optimizer.sjob_id, (
            f"Missconfiguration: slurm_job.*.bash "
            f"SJOB_ID={sjob_id} != cfg.hparam_optimizer.sjob_id={cfg.hparam_optimizer.sjob_id}"
        )

    # .... Execute multirun .......................................................................
    run_pytorch_check(cfg)

    return random.random()


if __name__ == "__main__":
    hyperparam_opt_pipeline()
