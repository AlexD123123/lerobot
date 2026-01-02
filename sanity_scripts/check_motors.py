from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode

# make sure to run before running the script:
#> sudo chmod 666 /dev/ttyACM0
#> sudo chmod 666 /dev/ttyACM1
import scservo_sdk as scs

ports = ["/dev/ttyACM0", "/dev/ttyACM1"]
baudrate = 1000000

for port in ports:
    print(f"Port {port}:")
    # Create port handler and packet handler
    port_handler = scs.PortHandler(port)
    packet_handler = scs.PacketHandler(1.0)  # Protocol 1.0 for STS servos

    if port_handler.openPort():
        if port_handler.setBaudRate(baudrate):
            print("Scanning for motors with IDs 1-6...")
            for motor_id in range(1, 7):
                model_number, result, error = packet_handler.ping(port_handler, motor_id)
                if result == scs.COMM_SUCCESS:
                    print(f"  ✓ Found motor ID {motor_id}")
                else:
                    print(f"  ✗ No motor at ID {motor_id}")
        port_handler.closePort()