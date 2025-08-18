#!/usr/bin/env python3

def verify_pytorch_install() -> None:
    """
    Minimal pytorch install verification. If the consol print a tensor like this, you're good

        tensor([[0.3380, 0.3845, 0.3217],
                [0.8337, 0.9050, 0.2650],
                [0.2979, 0.7141, 0.9069],
                [0.1449, 0.1132, 0.1375],
                [0.4675, 0.3947, 0.1426]])

    Ref: https://pytorch.org/get-started/locally/

    :return: None
    """
    print("\nStart Pytorch install check")

    try:
        import torch

        print(f"PyTorch version:    {torch.__version__}")

        x = torch.rand(5, 3)
        print("\n", x)
        print("\nPyTorch install is good to go!\n")
    except ImportError as e:
        raise Warning(f"PyTorch is not availablw! \n{e}")
    except Exception as e:
        # Note: The exception scope is large on purpose
        raise Warning(f"Something is wrong with PyTorch. \n{e}")

    return None


def verify_pytorch_cuda_install() -> None:
    """
    Minimal pytorch<->CUDA install verification. If the consol print a tensor like this,
    you're good

        tensor([[0.3380, 0.3845, 0.3217],
                [0.8337, 0.9050, 0.2650],
                [0.2979, 0.7141, 0.9069],
                [0.1449, 0.1132, 0.1375],
                [0.4675, 0.3947, 0.1426]])

    Ref: https://pytorch.org/get-started/locally/

    :return: None
    """
    print("\nStart Pytorch<->CUDA install check")

    try:
        import torch

        cuda_is_available = torch.cuda.is_available()

        if cuda_is_available:
            print(
                    f"CUDA available: {torch.cuda.is_available()}\n"
                    f"CUDA version: {torch.version.cuda}\n"
                    f"GPU device: {torch.cuda.get_device_name(None)}\n"
                    f"GPU device properties: {torch.cuda.get_device_properties(None)}\n"
                    f"GPU compute capability: {torch.cuda.get_device_capability(None)}\n"
                    f"Torch compiled for CUDA architecture: {torch.cuda.get_arch_list()}"
                    )

            x = torch.rand(5, 3).cuda()
            print("\n", x)
            print("\nPyTorch can access CUDA\n")
        else:
            print("Can't check PyTorch<->CUDA install.\n")
            raise ResourceWarning("CUDA is NOT available on this computer\n")
    except ImportError as e:
        raise Warning(f"PyTorch is not availablw! \n{e}")
    except ResourceWarning as ie:
        raise
    except Exception as e:
        # Note: The exception scope is large on purpose
        raise Warning(f"Something is wrong with PyTorch. \n{e}")

    return None


if __name__ == "__main__":
    verify_pytorch_install()
    verify_pytorch_cuda_install()
