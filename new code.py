import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor.servo import Servo
import numpy as np

# Predefined constants
a1 = 100  # mm
a2 = 15  # mm
a3 = 120  # mm
a4 = 117  # mm
d = 110  # mm

# Function to calculate angles for a given position
def theta(z032, r1):
    r2 = np.sqrt((z032 - a1)**2 + (r1 - a2)**2)
    psi3 = np.arccos((a3**2 + a4**2 - r2**2) / (2 * a3 * a4))
    print(f"psi3 is {psi3}")
    T3 = np.pi - psi3
    psi1 = np.arccos((a3**2 + r2**2 - a4**2) / (2 * a3 * r2))
    psi2 = np.arctan((z032 - a1) / (r1 - a2))
    T2 = psi2 - psi1
    return T2, T3, psi2, psi1

# Generate the array of angles
data = []
for i in range(100, 200, 10):
    x031 = i
    y031 = 1
    z031 = 25  # keep it to -19mm only for pen contact
    z032 = z031 + d
    r1 = np.sqrt(x031**2 + y031**2)
    T1 = np.arctan2(y031, x031)  # radians

    if r1 <= 190 and z032 > a1:
        T2, T3, psi2, psi1 = theta(z032, r1)
        if T2 < 0.436332:
            T2 = psi2 + psi1
            T3 *= -1
            T4 = -(T2 + T3 + (np.pi / 2))
        else:
            T4 = -(T2 + T3 + (np.pi / 2))
    elif r1 > 190:
        r1new = r1
        r1 = 190
        T = np.arcsin((r1new - r1) / d)
        z032 = z031 + d * np.cos(T)
        T2, T3, psi2, psi1 = theta(z032, r1)
        if T2 < 0.436332:
            T2 = psi2 + psi1
            T3 *= -1
            T4 = -(T2 + T3 + (np.pi / 2) - T)
        else:
            T4 = -((np.pi / 2) - T + T2 + T3)
    else:
        continue  # Skip invalid positions

    Theta1 = np.degrees(T1)
    Theta2 = np.degrees(T2)
    Theta3 = np.degrees(T3)
    Theta4 = np.degrees(T4)
    data.append([Theta1, Theta2, Theta3, Theta4])

array = np.array(data)
print(array)
# Initialize I2C bus and PCA9685 module
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Initialize servos
servo_base = Servo(pca.channels[0], min_pulse=500, max_pulse=2500)
servo_shoulder = Servo(pca.channels[6], min_pulse=500, max_pulse=2500)
servo_wrist = Servo(pca.channels[2], min_pulse=500, max_pulse=2500)
servo_gripper_roll = Servo(pca.channels[8], min_pulse=500, max_pulse=2500)
servo_gripper_pitch = Servo(pca.channels[4], min_pulse=500, max_pulse=2500)
servo_gripper_open_close = Servo(pca.channels[5], min_pulse=500, max_pulse=2500)

# Function to smoothly move all servos simultaneously
def move_arm_to_position_simultaneous(angles, step_delay=0.02, steps=50):
    """
    Smoothly moves all servos to the specified angles simultaneously,
    ensuring intermediate angles stay within range.
    """
    current_angles = [
        servo_base.angle or 90,
        servo_shoulder.angle or 90,
        servo_wrist.angle or 90,
        servo_gripper_roll.angle or 90,
        90,  # Default for gripper_pitch
        servo_gripper_open_close.angle or 90
    ]

    step_angles = [
        (target - current) / steps
        for target, current in zip(angles, current_angles)
    ]

    for step in range(steps):
        # Calculate new angles for this step
        base_angle = max(0, min(180, current_angles[0] + step * step_angles[0]))
        shoulder_angle = max(0, min(180, current_angles[1] + step * step_angles[1]))
        wrist_angle = max(0, min(180, current_angles[2] + step * step_angles[2]))
        roll_angle = max(0, min(180, current_angles[3] + step * step_angles[3]))
        gripper_angle = max(0, min(180, current_angles[5] + step * step_angles[5]))

        # Set the servo angles
        servo_base.angle = base_angle
        servo_shoulder.angle = shoulder_angle
        servo_wrist.angle = wrist_angle
        servo_gripper_roll.angle = roll_angle
        servo_gripper_open_close.angle = gripper_angle
        
        time.sleep(step_delay)

    # Ensure final target positions are set
    servo_base.angle = angles[0]
    servo_shoulder.angle = angles[1]
    servo_wrist.angle = angles[2]
    servo_gripper_roll.angle = angles[3]
    servo_gripper_open_close.angle = angles[5]

if __name__ == "__main__":
    try:
        for row in array:
            # Calculate dynamic pickup position angles
            theta1 = 30 + (row[0])
            theta2 = (row[1]) + 8
            theta3 = -row[2]
            theta4 = -row[3]-12 
            
            pickup_position_angles = [theta1, theta2, theta3, theta4, 0, 70]  # Last angle is gripper

            print(f"Moving to pickup position: {pickup_position_angles}")
            move_arm_to_position_simultaneous(pickup_position_angles)

            print("Closing gripper to pick up object...")
            time.sleep(2)  # Delay before next iteration

        print("Pick and place sequence complete.")
    except KeyboardInterrupt:
        print("\nProcess interrupted by the user.")
        move_arm_to_position_simultaneous([30, 98, 0, 10, 0, 90])  # Return to home position if interrupted
        pca.deinit()
    except Exception as e:
        print(f"An error occurred: {e}")
        move_arm_to_position_simultaneous([30, 98, 0, 10, 0, 90])  # Return to home position in case of an error
    finally:
        pca.deinit()
