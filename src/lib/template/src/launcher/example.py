# coding=utf-8

from dnp_example.try_pytorch import verify_pytorch_install, verify_pytorch_cuda_install
import torch

def run_pytorch_check() -> None:

    verify_pytorch_install()
    if torch.cuda.is_available():
        verify_pytorch_cuda_install()

    return None


if __name__ == "__main__":
    run_pytorch_check()
