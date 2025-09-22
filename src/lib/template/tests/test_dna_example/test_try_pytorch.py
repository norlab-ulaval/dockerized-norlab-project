#!/usr/bin/env python3

import pytest

from dna_example.try_pytorch import verify_pytorch_install, verify_pytorch_cuda_install

from torch import cuda


def test_verify_pytorch_install_PASS():
    verify_pytorch_install()


@pytest.mark.skipif(
        (not cuda.is_available()),
        reason="Cuda is not suported on this host",
        )
def test_verify_pytorch_cuda_install():
    verify_pytorch_cuda_install()
