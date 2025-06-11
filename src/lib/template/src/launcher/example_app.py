# coding=utf-8

import omegaconf
import hydra

from dnp_example.try_pytorch import verify_pytorch_install, verify_pytorch_cuda_install
import torch

@hydra.main(config_path="configs", config_name="example_app", version_base=None)
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
