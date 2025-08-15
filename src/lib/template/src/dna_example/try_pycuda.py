#!/usr/bin/env python3

def verify_pycuda_install() -> None:
    try:
        import pycuda.driver as drv
        import pycuda.autoinit  # This initializes PyCUDA and sets up a context

        # Get the number of CUDA-enabled devices
        num_devices = drv.Device.count()
        print(f"{num_devices} device(s) found.")

        # Iterate through each device to retrieve its properties
        for i in range(num_devices):
            dev = drv.Device(i)
            print(f"\nDevice #{i}: {dev.name()}")
            print(
                    f"  Compute Capability: {dev.compute_capability()[0]}."
                    f"{dev.compute_capability()[1]}")

            # Total Memory in GB
            total_memory_bytes = dev.total_memory()
            total_memory_gb = total_memory_bytes / (1024 ** 3)
            print(f"  Total Memory: {total_memory_gb:.2f} GB")

            print(
                    f"  Number of Multiprocessors: "
                    f"{dev.get_attribute(drv.device_attribute.MULTIPROCESSOR_COUNT)}")
            print(
                    f"  Max Threads Per Block: "
                    f"{dev.get_attribute(drv.device_attribute.MAX_THREADS_PER_BLOCK)}")
    except ImportError as e:
        raise Warning(f"PyCuda is not availablw! \n{e}")
    except Exception as e:
        # Note: The exception scope is large on purpose
        raise Warning(f"Something is wrong with PyCuda. \n{e}")

    return None


if __name__ == "__main__":
    verify_pycuda_install()
