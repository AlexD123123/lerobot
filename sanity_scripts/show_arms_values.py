from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode

# Leader arm - read raw positions
leader = FeetechMotorsBus(
    port="/dev/ttyACM0",
    motors={
        "shoulder_pan": Motor(id=1, model="sts3215", norm_mode=MotorNormMode.DEGREES),
        "shoulder_lift": Motor(id=2, model="sts3215", norm_mode=MotorNormMode.DEGREES),
        "elbow_flex": Motor(id=3, model="sts3215", norm_mode=MotorNormMode.DEGREES),
        "wrist_flex": Motor(id=4, model="sts3215", norm_mode=MotorNormMode.DEGREES),
        "wrist_roll": Motor(id=5, model="sts3215", norm_mode=MotorNormMode.DEGREES),
        "gripper": Motor(id=6, model="sts3215", norm_mode=MotorNormMode.DEGREES),
    }
)

leader.connect()
print("LEADER ARM - Raw positions:")
for motor_name, motor in leader.motors.items():
    # Read directly using the SDK
    import scservo_sdk as scs
    value, result, error = leader.packet_handler.read2ByteTxRx(
        leader.port_handler, motor.id, 56  # 56 is Present_Position address
    )
    if result == scs.COMM_SUCCESS:
        print(f"  {motor_name} (ID {motor.id}): {value}")
    else:
        print(f"  {motor_name} (ID {motor.id}): ERROR")
leader.disconnect()

print("\n" + "="*60 + "\n")

# Follower arm
follower = FeetechMotorsBus(
    port="/dev/ttyACM1",
    motors={
        "shoulder_pan": Motor(id=1, model="sts3215", norm_mode=MotorNormMode.DEGREES),
        "shoulder_lift": Motor(id=2, model="sts3215", norm_mode=MotorNormMode.DEGREES),
        "elbow_flex": Motor(id=3, model="sts3215", norm_mode=MotorNormMode.DEGREES),
        "wrist_flex": Motor(id=4, model="sts3215", norm_mode=MotorNormMode.DEGREES),
        "wrist_roll": Motor(id=5, model="sts3215", norm_mode=MotorNormMode.DEGREES),
        "gripper": Motor(id=6, model="sts3215", norm_mode=MotorNormMode.DEGREES),
    }
)

follower.connect()
print("FOLLOWER ARM - Raw positions:")
for motor_name, motor in follower.motors.items():
    value, result, error = follower.packet_handler.read2ByteTxRx(
        follower.port_handler, motor.id, 56
    )
    if result == scs.COMM_SUCCESS:
        print(f"  {motor_name} (ID {motor.id}): {value}")
    else:
        print(f"  {motor_name} (ID {motor.id}): ERROR")
follower.disconnect()