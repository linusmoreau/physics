import math
import functools

desired_accuracy = 0.1
step = 0.0001        # s
p_air = 1.2         # kg/m^3
g = -9.8            # m/s/s
kick = 50           # kg * m


class Sphere:
    air_const = 0.1

    def __init__(self, radius=0.2, mass=2, position=None, momentum=None, forces=None):
        """radius: metres, mass: kg"""
        self.radius = radius
        self.mass = mass
        if position is None:
            self.position = [0, 0, 0]
        if momentum is None:
            self.momentum = [0, 0, 0]
        if forces is None:
            self._forces = []

    def add_force(self, force):
        self._forces.append(force)

    def kick(self, impulse):
        for i in range(len(self.momentum)):
            self.momentum[i] += impulse[i]

    def get_net_force(self):
        net = [0, 0, 0]
        for f in self._forces:
            if type(f).__name__ == "partial":
                f = f()
            for i in range(len(f)):
                net[i] += f[i]
        return net

    def get_air_resistance(self):
        # returns air resistance force vector
        velocity = self.get_velocity()
        unit_vect, mag = unit_vector(velocity)
        force_mag = -self.air_const * p_air * math.pi * self.radius ** 2 * mag ** 2
        return [unit_vect[i] * force_mag for i in range(len(velocity))]

    def get_velocity(self):
        # This follows Newtonian physics (does not behave properly near c)
        return [dim / self.mass for dim in self.momentum]

    def update_momentum(self, dt=step):
        net_force = self.get_net_force()
        for i in range(len(self.momentum)):
            self.momentum[i] += net_force[i] * dt

    def update_position(self, dt=step):
        # This follows Newtonian physics (does not behave properly near c)
        for i in range(len(self.position)):
            self.position[i] += self.momentum[i] / self.mass * dt


def unit_vector(vector):
    mag = sum([dim**2 for dim in vector])**(1/2)
    unit = [vector[i] / mag for i in range(len(vector))]
    return unit, mag


def outlier(data, max_is_out, attr):
    """returns index of outlier for attr in data (min if max_is_out=False, max if max_is_out=True)"""
    out = 0
    for i in range(1, len(data)):
        if max_is_out == (data[i][attr] > data[out][attr]):
            out = i
    return out


def get_vector(mag, angle):
    return [mag * math.cos(angle), mag * math.sin(angle), 0]


def display_result(angle, position, time):
    print("Angle:", round(math.degrees(angle), 1), "degrees;   ",
          "Distance:", round(position, 2), "metres;   ",
          "Time:", round(time, 2), "seconds")


results = []
max_ang = math.pi / 2
min_ang = 0
iters_per_run = 10

times = 0
decreased = False
iter_num = 0
finished = False
while not finished:
    # Determine angle
    angle = min_ang + ((max_ang - min_ang) / iters_per_run) * iter_num

    # Make ball
    ball = Sphere()
    ball.kick(get_vector(kick, angle))

    # Add forces
    ball.add_force((0, g, 0))
    ball.add_force(functools.partial(Sphere.get_air_resistance, ball))

    # Iteration loop
    time = 0
    while True:
        time += step
        ball.update_momentum()
        ball.update_position()
        if ball.position[1] <= 0:
            results.append({"angle": angle, "distance": ball.position[0]})
            display_result(angle, ball.position[0], time)
            break

    iter_num += 1
    if len(results) > 1 and results[-2]["distance"] > results[-1]["distance"]:
        times += iter_num
        iter_num = 1
        if len(results) > 2:
            min_ang = results[-3]["angle"]
            max_ang = results[-1]["angle"]
            results = [results[-3]]
        else:
            min_ang = results[0]["angle"]
            max_ang = results[1]["angle"]
            results = [results[0]]
        if max_ang - min_ang < desired_accuracy / 10:
            finished = True
            break
