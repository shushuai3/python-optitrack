# Code to fly multiple Crazyflies with optitrack

import sys
import time
from NatNetClient import NatNetClient

import math
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

import numpy as np

# left-down: [3.46, 3.35]; right-down: [-0.01, 3.33]
# left-up: [3.46, -0.16]; right-up: [-0.01, -0.16]
# left: positive x; down: positive y


# Change uris according to your setup
# URI1 = 'radio://0/80/2M/E7E7E7E7E2'
URI2 = 'radio://0/80/2M/E7E7E7E7E0'

# Params link optitrack and cflib
# params1 = {'d': 0.0, 'z': 0.3}
params2 = {'vx': 0.0, 'vy': 0.0}

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
    fs = 10
    fsi = 1.0 / fs

    # Base altitude in meters
    base = 0.15

    poshold(cf, 2, base)

    ramp = fs * 2
    for r in range(ramp):
        cf.commander.send_hover_setpoint(0, 0, 0, base + r * (0.4 - base) / ramp)
        time.sleep(fsi)

    poshold(cf, 2, 0.4)

    for _ in range(2):
        # The time for one revolution
        circle_time = 120

        steps = circle_time * fs
        for _ in range(steps):
            vx, vy = params['vx'], params['vy']
            cf.commander.send_hover_setpoint(vx, vy, 0, 0.4)
            time.sleep(fsi)

    poshold(cf, 2, 0.4)

    for r in range(ramp):
        cf.commander.send_hover_setpoint(0, 0, 0,
                                         base + (ramp - r) * (0.4 - base) / ramp)
        time.sleep(fsi)

    poshold(cf, 1, base)

    cf.commander.send_stop_setpoint()

position = [[2.68, 2.54]]
nb_agent = len(position)
velocity_array = np.random.uniform(low=0.2, high=0.5, size=(nb_agent, 2))
velocity = velocity_array.tolist()

def normalize(v):
    """ Normalize a vector to length 1. """
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return 0.5 * v / norm

def find_neighbors(self_index, position, distance):
    nb_agent = len(position)
    neighbor_id_list = []
    for i in range(nb_agent):
        if i != self_index:
            # Calculate neighbors within the distance
            pos_nei_array = np.array(position[i])
            pos_my_array = np.array(position[self_index])
            if np.linalg.norm(pos_nei_array - pos_my_array) < distance:
                neighbor_id_list.append(i)
    # neighbor_position = [position[i] for i in neighbor_id_list]
    return neighbor_id_list

def flock_control(position, velocity):
    # Code from https://agentpy.readthedocs.io/en/latest/agentpy_flocking.html
    # Create return value
    nb_agent = len(position)
    velocity_new = [[0.0, 0.0] for i in range(nb_agent)]

    for i in range(nb_agent):
        self_position = np.array(position[i])
        self_velocity = np.array(velocity[i])

        # Rule 1 - Cohesion
        cohesion_strength = 0.005
        nbs = find_neighbors(i, position, distance=1.3)
        nbs_len = len(nbs)
        nbs_pos_array = np.array([position[j] for j in nbs])
        nbs_vec_array = np.array([velocity[j] for j in nbs])
        if nbs_len > 0:
            center = np.sum(nbs_pos_array, 0) / nbs_len
            v1 = (center - self_position) * cohesion_strength
        else:
            v1 = np.zeros(2)
        
        # Rule 2 - Seperation
        seperation_strength = 1.0
        v2 = np.zeros(2)
        for nb in find_neighbors(i, position, distance=0.5):
            v2 -= np.array(position[nb]) - self_position
        v2 *= seperation_strength
        
        # Rule 3 - Alignment
        alignment_strength = 0.01
        if nbs_len > 0:
            average_v = np.sum(nbs_vec_array, 0) / nbs_len
            v3 = (average_v - self_velocity) * alignment_strength
        else:
            v3 = np.zeros(2)
        
        # Rule 4 - Borders
        v4 = np.zeros(2)
        # !boarder value measured by optitrack!
        boarder = [-0.01, 3.36, -0.16, 3.35]
        boarder_distance = 0.3
        boarder_strength = 1.0
        if self_position[0] < boarder_distance + boarder[0]:
            v4[0] += boarder_strength
        elif self_position[0] > boarder[1] - boarder_distance:
            v4[0] -= boarder_strength
        if self_position[1] < boarder_distance + boarder[2]:
            v4[1] += boarder_strength
        elif self_position[1] > boarder[3] - boarder_distance:
            v4[1] -= boarder_strength

        # Update velocity
        velocity_cmd = v1 + v2 + v3 + v4 + self_velocity
        velocity_cmd_norm = normalize(velocity_cmd)
        velocity_new[i][0] = velocity_cmd_norm[0]
        velocity_new[i][1] = velocity_cmd_norm[1]

    return velocity_new

# This is called once per mocap frame.
def receive_new_frame(data_dict):
    # pass
    global params2, position, velocity
    print(data_dict["frame_number"])
    print(data_dict["labeled_marker_count"])
    a = data_dict["labeled_marker_list"]
    # print(a[0].pos)
    b = a[0].pos
    position[0] = b[0:2]
    velocity = flock_control(position, velocity)
    params2['vx'], params2['vy'] = velocity[0][0], velocity[0][1]

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