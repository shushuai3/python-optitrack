import cv2
import numpy as np

meter2pix = 100
radius = 0.07

agent_icon = cv2.imread("agent.jpg") / 255.0
agent_icon = cv2.resize(agent_icon, (2 * int(meter2pix * radius), 2 * int(meter2pix * radius)))
dt = 0.01

def normalize(v):
    """ Normalize a vector to length 1. """
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

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
        alignment_strength = 0.03
        if nbs_len > 0:
            average_v = np.sum(nbs_vec_array, 0) / nbs_len
            v3 = (average_v - self_velocity) * alignment_strength
        else:
            v3 = np.zeros(2)
        
        # Rule 4 - Borders
        v4 = np.zeros(2)
        boarder_distance = 0.5
        boarder_strength = 1.0
        size = 5.0
        for j in range(2):
            if self_position[j] < boarder_distance:
                v4[j] += boarder_strength
            elif self_position[j] > size - boarder_distance:
                v4[j] -= boarder_strength
        
        # Update velocity
        velocity_cmd = v1 + v2 + v3 + v4 + self_velocity
        velocity_cmd_norm = normalize(velocity_cmd)
        velocity_new[i][0] = velocity_cmd_norm[0]
        velocity_new[i][1] = velocity_cmd_norm[1]

    return velocity_new

nb_agent = 10
position_array = np.random.uniform(low=0.2, high=4.8, size=(nb_agent, 2))
position = position_array.tolist()

# velocity = [[0.0, 0.0] for i in range(nb_agent)]
velocity_array = np.random.uniform(low=0.2, high=1.0, size=(nb_agent, 2))
velocity = velocity_array.tolist()

while True:
    canvas = np.ones((500, 500, 3)) * 1

    velocity = flock_control(position, velocity)

    for i in range(nb_agent):
        # velocity commands
        # velocity[i] = [1.0, 1.0]

        # position update
        position[i][0] += velocity[i][0] * dt
        position[i][1] += velocity[i][1] * dt

        # draw with opencv
        [x, y] = position[i]
        x_pixel = int(meter2pix * x) - int(meter2pix * radius)
        y_pixel = int(meter2pix * y) - int(meter2pix * radius)
        canvas[y_pixel: y_pixel + agent_icon.shape[1], x_pixel: x_pixel + agent_icon.shape[0]] = agent_icon

    # show with opencv
    cv2.imshow("Game", canvas)
    cv2.waitKey(10)