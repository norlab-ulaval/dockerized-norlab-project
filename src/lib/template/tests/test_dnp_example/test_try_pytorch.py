#!/usr/bin/env python3

import pytest

from dnp_example.try_pytorch import verify_pytorch_install, verify_pytorch_cuda_install

from torch import cuda


def test_verify_pytorch_install_PASS():
    verify_pytorch_install()
    return None


@pytest.mark.skipif(
    (not cuda.is_available()),
    reason="Cuda is not suported on this host",
)
def test_verify_pytorch_cuda_install():
    # (Priority) ToDo: refactor for Dockerized-NorLab 2.0
    # print(os.environ)
    verify_pytorch_cuda_install()
    return None
