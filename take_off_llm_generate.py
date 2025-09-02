import setup_path
import airsim
import sys
import time

# For high speed ascent and descent on PX4 you may need to set these properties:
# param set MPC_Z_VEL_MAX_UP 5
# param set MPC_Z_VEL_MAX_DN 5

z = 20  # Target altitude

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync().join()
else:
    print("already flying...")
    client.hoverAsync().join()

# Takeoff to 20 meters
print("Taking off to {} meters...".format(z))
client.moveToZAsync(-z, 5).join()
client.hoverAsync().join()
time.sleep(1)

# Check altitude and hover/land
while True:
    current_z = -client.getMultirotorState().kinematics_estimated.position.z
    print(f"Current altitude: {current_z:.2f} meters")

    if abs(current_z - 10) < 1:  # Within 1 meter of 10 meters
        print("Hovering at 10 meters for 5 seconds...")
        client.hoverAsync().join()
        time.sleep(5)
        print("Continuing to takeoff...")
        client.moveToZAsync(-z, 5).join()
        client.hoverAsync().join()
        time.sleep(1)

    elif abs(current_z - 20) < 1:  # Within 1 meter of 20 meters
        print("Hovering at 20 meters for 3 seconds...")
        client.hoverAsync().join()
        time.sleep(3)
        print("Landing...")
        client.landAsync().join()
        break  # Exit the loop after landing

    time.sleep(1)  # Check altitude every second

print("disarming...")
client.armDisarm(False)
client.enableApiControl(False)
print("done.")