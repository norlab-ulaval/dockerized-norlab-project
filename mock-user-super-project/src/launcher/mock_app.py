# coding=utf-8

import omegaconf
import hydra

from tools.try_pytorch import verify_pytorch_install, verify_pytorch_cuda_install
import torch

@hydra.main(config_path="configs", config_name="pytorch_check", version_base=None)
def run_pytorch_check(cfg: omegaconf.DictConfig):

    if cfg.run_pytorch_check is True:
        verify_pytorch_install()
        if torch.cuda.is_available():
            verify_pytorch_cuda_install()
    else:
        print("Skip pytorch check")

    return None


if __name__ == "__main__":
    run_pytorch_check()
