#!/bin/bash

#apt-get update \
#  && apt-get upgrade -y \
#  && apt-get install -y \
#    "python3-opencv" \
#  && apt-get autoremove -y \
#  && apt-get clean \
#  && echo "Validate opencv install"\
#  && python3 -c "import cv2; print( f'Opencv version: {cv2.__version__}' )"

apt-get update \
  && apt-get upgrade -y \
  && apt-get install -y \
    vim \
  && apt-get autoremove -y \
  && apt-get clean

# TorchMetrics is a collection of 100+ PyTorch metrics implementations and an easy-to-use API to create custom metrics
# https://lightning.ai/docs/torchmetrics/stable/
pip3 install torchmetrics
python -c "import torchmetrics"

