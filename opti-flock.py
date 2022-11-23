# Code to fly multiple Crazyflies with optitrack

import sys
import time
from NatNetClient import NatNetClient

import math
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

# Change uris according to your setup
# URI1 = 'radio://0/80/2M/E7E7E7E7E2'
URI2 = 'radio://0/80/2M/E7E7E7E7E0'

# Params link optitrack and cflib
# params1 = {'d': 0.0, 'z': 0.3}
params2 = {'d': 0.0, 'z': 0.5}

uris = {
    # URI0,
    # URI1,
    URI2,
    # URI3,
    # URI4,
}

params = {
    # URI0: [params0],
    # URI1: [params1],
    URI2: [params2],
    # URI3: [params3],
    # URI4: [params4],
}

def poshold(cf, t, z):
    steps = t * 10

    for r in range(steps):
        cf.commander.send_hover_setpoint(0, 0, 0, z)
        time.sleep(0.1)

def run_sequence(scf, params):
    cf = scf.cf

    # Number of setpoints sent per second
    fs = 4
    fsi = 1.0 / fs

    # Compensation for unknown error :-(
    comp = 1.3

    # Base altitude in meters
    base = 0.15

    d = params['d']
    print(d)
    z = params['z']

    poshold(cf, 2, base)

    ramp = fs * 2
    for r in range(ramp):
        d = params['d']
        print("what tttttttttttttttttttttttttttttttt")
        print(d)
        cf.commander.send_hover_setpoint(0, 0, 0, base + r * (z - base) / ramp)
        time.sleep(fsi)

    poshold(cf, 2, z)

    for _ in range(2):
        # The time for one revolution
        circle_time = 8

        steps = circle_time * fs
        for _ in range(steps):
            cf.commander.send_hover_setpoint(d * comp * math.pi / circle_time,
                                             0, 360.0 / circle_time, z)
            time.sleep(fsi)

    poshold(cf, 2, z)

    for r in range(ramp):
        cf.commander.send_hover_setpoint(0, 0, 0,
                                         base + (ramp - r) * (z - base) / ramp)
        time.sleep(fsi)

    poshold(cf, 1, base)

    cf.commander.send_stop_setpoint()

# This is called once per mocap frame.
def receive_new_frame(data_dict):
    global params2
    params2['d'] += 0.0000001
    print(data_dict["frame_number"])
    print(data_dict["labeled_marker_count"])
    a = data_dict["labeled_marker_list"]
    print(a[0].pos)
    pass

if __name__ == "__main__":

    # Settings for the optitrack
    streaming_client = NatNetClient()
    streaming_client.set_server_address("192.168.0.249")
    streaming_client.new_frame_listener = receive_new_frame

    # Start up the streaming client, callbacks operate on a separate thread.
    is_running = streaming_client.run()
    if not is_running:
        print("ERROR: Could not start streaming client.")
        sys.exit(1)
    time.sleep(1)
    if streaming_client.connected() is False:
        print("ERROR: Could not connect.  Check that Motive streaming is on.")
        sys.exit(2)

    # Start drone control
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.reset_estimators()
        swarm.parallel(run_sequence, args_dict=params)

    # Main loop for control
    while True:
        time.sleep(1)
        print("here")