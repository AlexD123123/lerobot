from pyorbbecsdk import Context

try:
    ctx = Context()
    device_list = ctx.query_devices()
    print(f"Found {device_list.get_count()} devices")

    if device_list.get_count() > 0:
        for i in range(device_list.get_count()):
            device = device_list.get_device_by_index(i)
            info = device.get_device_info()
            print(f"Device {i}: {info.get_name()}")
    else:
        print("No devices found by Orbbec SDK")

except Exception as e:
    print(f"Error: {e}")
    import traceback

    traceback.print_exc()