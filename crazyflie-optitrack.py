# Test with motive 2.3.1 & crazyflie 2022.05
# Random flight in a confined area

import sys
import time
from NatNetClient import NatNetClient

# This is called once per mocap frame.
def receive_new_frame(data_dict):
    pass

# It is called once per rigid body per frame
def receive_rigid_body_frame( new_id, position, rotation ):
    # pass
    print("Received frame for rigid body", new_id," ",position," ",rotation )


if __name__ == "__main__":

    # Settings for the optitrack
    streaming_client = NatNetClient()
    streaming_client.set_server_address("192.168.0.202")
    streaming_client.new_frame_listener = receive_new_frame
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client, callbacks operate on a separate thread.
    is_running = streaming_client.run()
    if not is_running:
        print("ERROR: Could not start streaming client.")
        sys.exit(1)

    time.sleep(1)
    if streaming_client.connected() is False:
        print("ERROR: Could not connect.  Check that Motive streaming is on.")
        sys.exit(2)

    # Main loop for control
    while True:
        time.sleep(1)