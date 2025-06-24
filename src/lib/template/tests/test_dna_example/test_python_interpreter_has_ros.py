# coding=utf-8

import pytest
import os


def test_ros_python_module_import_ok():
    path = os.getenv("PATH")
    python_path = os.getenv("PYTHONPATH")
    ament_prefix_path = os.getenv("AMENT_PREFIX_PATH")

    # print(os.environ)
    # print("path:", path)
    # print("python_path:", python_path)
    # print("ament_prefix_path:", ament_prefix_path)

    assert path is not None
    assert python_path is not None
    assert ament_prefix_path is not None

    import rclpy

    print(rclpy)


def test_pytest_ini_config_file():
    env_var = os.getenv("LOADED_ENV_FROM_PYTEST_INI")
    assert env_var is not None
    assert env_var == "loaded like a boss"


def test_ROS_environment_variable_are_sourced():
    ament_prefix_path = os.getenv("AMENT_PREFIX_PATH")
    print(ament_prefix_path)
    assert ament_prefix_path is not None
    assert os.getenv("COLCON_PREFIX_PATH") is not None
    assert os.getenv("ROS_ROOT") is not None
    assert os.getenv("ROS_DISTRO") is not None
    assert os.getenv("PYTHONPATH") is not None


# # ToDo: assessment >> maybe require '.env.dn_expose_IamRedLeader' be sourced
# #       in Dockerfile.ci-tests.native
# @pytest.mark.skipif(
#     (os.getenv("DISPLAY") is None),
#     reason="No display available",
# )
# def test_display_forwarding_environment_variable_are_sourced():
#     assert os.getenv("LIBGL_ALWAYS_INDIRECT") is not None
#     assert os.getenv("QT_X11_NO_MITSHM") is not None
