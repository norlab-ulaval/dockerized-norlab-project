#!/usr/bin/env python3


# ToDo: refactor out to DN deep learning related images
def verify_pytorch_install() -> None:
    """
    Minimal pytorch install verification. If the consol print a tensor like this, your good

        tensor([[0.3380, 0.3845, 0.3217],
                [0.8337, 0.9050, 0.2650],
                [0.2979, 0.7141, 0.9069],
                [0.1449, 0.1132, 0.1375],
                [0.4675, 0.3947, 0.1426]])

    Ref: https://pytorch.org/get-started/locally/

    :return: None
    """

    # (NICE TO HAVE) ToDo: refactor >> with `tools.console_tools.message.consol_msg()`
    print("\n> Start Pytorch install check")

    try:
        import torch

        print(f"> PyTorch version:    {torch.__version__}")

        x = torch.rand(5, 3)
        print("\n", x)
        print("\n> PyTorch install is good to go!\n")
    except Exception as e:
        # Note: The exception scope is large on purpose
        raise Exception(
            "> Something is wrong with PyTorch. It's probably the Dockerized-NorLab python "
            "interpreter"
        ) from e

    return None


# ToDo: refactor out to DN deep learning related images
def verify_pytorch_cuda_install() -> None:
    """
    Minimal pytorch<->CUDA install verification. If the consol print a tensor like this, your good

        tensor([[0.3380, 0.3845, 0.3217],
                [0.8337, 0.9050, 0.2650],
                [0.2979, 0.7141, 0.9069],
                [0.1449, 0.1132, 0.1375],
                [0.4675, 0.3947, 0.1426]])

    Ref: https://pytorch.org/get-started/locally/

    :return: None
    """
    # (NICE TO HAVE) ToDo: refactor >> with `tools.console_tools.message.consol_msg()`
    print("\n> Start Pytorch<->CUDA install check")

    import torch

    cuda_is_available = torch.cuda.is_available()

    # assert cuda_is_available, "> CUDA is not available to PyTorch\n"
    # print(f"> CUDA is available:  {cuda_is_available}")

    if cuda_is_available:
        try:
            print(f"> CUDA is available")
            # print(f'cuDNN version:      {torch.backends.cudnn.version()}')

            x = torch.rand(5, 3).cuda()
            print("\n", x)
            print("\n> PyTorch can access CUDA\n")
        except Exception as e:
            # Note: The exception scope is large on purpose
            raise Exception(f"> Something is wrong with the PyTorch<->CUDA install.\n{e}")

    else:
        print("> > Can't check PyTorch<->CUDA install.\n")
        raise ResourceWarning("> CUDA is NOT available on this computer\n")

    return None


if __name__ == "__main__":
    verify_pytorch_install()
    verify_pytorch_cuda_install()
