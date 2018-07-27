import numpy as np
import yaml
import pydmps

class RollDMP():
    def __init__(self, file_name, n_dmps=6, n_bfs=50):
        weight = self.load_weights(file_name)
        self.dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=n_dmps, n_bfs=n_bfs,
                                                     dt=0.001, ay=np.ones(n_dmps)*10.0, w=weight)

    def roll(self, goal, initial_pos, tau):
        self.pos, self.vel, self.acc = self.dmp.rollout(goal=goal, y0=initial_pos, tau=tau)
        return self.pos, self.vel, self.acc

    def load_weights(self, file_name):
        with open(file_name) as f:
            loadeddict = yaml.load(f)
        x = loadeddict.get('x')
        y = loadeddict.get('y')
        z = loadeddict.get('z')
        roll = loadeddict.get('roll')
        pitch = loadeddict.get('pitch')
        yaw = loadeddict.get('yaw')

        x = np.array(x)
        y = np.array(y)
        z = np.array(z)
        roll = np.array(roll)
        pitch = np.array(pitch)
        yaw = np.array(yaw)

        weights = x
        weights = np.vstack((weights, y))
        weights = np.vstack((weights, z))
        weights = np.vstack((weights, roll))
        weights = np.vstack((weights, pitch))
        weights = np.vstack((weights, yaw))

        return weights
