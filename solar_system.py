import pybullet as p
import pybullet_data
import time
import math
import random


def main():
    """Main function to run the solar system simulation."""
    # Connect to the simulator in GUI mode
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)  # Disable gravity, as we calculate it manually

    # System parameters
    G = 1e-2  # Scaled gravitational constant
    dt = 1.0 / 240.0  # Time step

    # Create the sun (central body)
    sun_mass = 1e4
    sun_radius = 1.0
    sun_position = [0, 0, 0]
    sun_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=sun_radius)
    sun_visual = p.createVisualShape(
        p.GEOM_SPHERE, radius=sun_radius, rgbaColor=[1, 1, 0, 1]
    )
    sun_id = p.createMultiBody(
        sun_mass, sun_collision, sun_visual, basePosition=sun_position
    )
    p.resetBaseVelocity(sun_id, [0, 0, 0])

    # Create planets with initial positions and velocities in circular orbits
    num_planets = 5
    planets = []
    planet_data = {}  # Store position, velocity, and mass of each planet

    for i in range(num_planets):
        # Distribute planets in circular orbits around the sun
        angle = 2 * math.pi * i / num_planets
        distance = random.uniform(5, 15)
        pos = [distance * math.cos(angle), distance * math.sin(angle), 0]

        # Create the sphere representing the planet
        planet_mass = 5
        planet_radius = 0.3
        col_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=planet_radius)
        color = [random.random(), random.random(), random.random(), 1]
        vis_shape = p.createVisualShape(
            p.GEOM_SPHERE, radius=planet_radius, rgbaColor=color
        )
        planet_id = p.createMultiBody(
            planet_mass, col_shape, vis_shape, basePosition=pos
        )

        # Calculate circular orbital velocity: v = sqrt(G * M_sun / r)
        v_magnitude = math.sqrt(G * sun_mass / distance)
        # Velocity is perpendicular to the radius (for circular orbit)
        vel = [-v_magnitude * math.sin(angle), v_magnitude * math.cos(angle), 0]

        # Store planet data
        planet_data[planet_id] = {"pos": pos, "vel": vel, "mass": planet_mass}
        planets.append(planet_id)

    # Create an asteroid with an orbit influenced by the sun
    asteroid_mass = 1
    asteroid_radius = 0.2
    asteroid_distance = 10  # Initial distance of the asteroid from the sun
    asteroid_angle = random.uniform(0, 2 * math.pi)
    asteroid_position = [
        asteroid_distance * math.cos(asteroid_angle),
        asteroid_distance * math.sin(asteroid_angle),
        0,
    ]
    asteroid_velocity = [0, 0, 0]  # Initially no velocity

    # Calculate the circular orbital velocity of the asteroid
    v_magnitude_asteroid = math.sqrt(G * sun_mass / asteroid_distance)
    asteroid_velocity = [
        -v_magnitude_asteroid * math.sin(asteroid_angle),
        v_magnitude_asteroid * math.cos(asteroid_angle),
        0,
    ]

    # Create the asteroid in PyBullet
    asteroid_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=asteroid_radius)
    asteroid_visual = p.createVisualShape(
        p.GEOM_SPHERE, radius=asteroid_radius, rgbaColor=[0.5, 0.5, 0.5, 1]
    )
    asteroid_id = p.createMultiBody(
        asteroid_mass,
        asteroid_collision,
        asteroid_visual,
        basePosition=asteroid_position,
    )
    planet_data[asteroid_id] = {
        "pos": asteroid_position,
        "vel": asteroid_velocity,
        "mass": asteroid_mass,
    }
    planets.append(asteroid_id)

    # Configure the camera to observe the system
    camera_distance = 30
    camera_yaw = 90
    camera_pitch = -30
    camera_target_position = [0, 0, 0]

    def update_camera():
        """Update the camera position and orientation."""
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
            cameraTargetPosition=camera_target_position,
        )

    update_camera()

    def gravitational_acceleration(pos):
        """Calculate gravitational acceleration due to the sun at a given position.

        Args:
            pos (list): Position vector [x, y, z].

        Returns:
            list: Gravitational acceleration vector [ax, ay, az].
        """
        # Vector from the planet to the sun
        r_vec = [sun_position[i] - pos[i] for i in range(3)]
        r_mag = math.sqrt(sum(x * x for x in r_vec))
        if r_mag < 1e-3:
            return [0, 0, 0]
        # Acceleration = G * M_sun / r² * (r_vec / r_mag)
        a = [G * sun_mass / (r_mag**2) * (r_vec[i] / r_mag) for i in range(3)]
        return a

    def gravitational_assistance(pos, vel):
        """Apply additional acceleration when the asteroid approaches the sun.

        Args:
            pos (list): Position vector [x, y, z].
            vel (list): Velocity vector [vx, vy, vz].

        Returns:
            list: Updated velocity vector [vx, vy, vz].
        """
        # Calculate the distance to the sun
        r_vec = [sun_position[i] - pos[i] for i in range(3)]
        r_mag = math.sqrt(sum(x * x for x in r_vec))

        # If the asteroid is close enough to the sun, increase its velocity
        if r_mag < 5.0:  # Distance threshold where extra acceleration is activated
            # Calculate an extra acceleration factor
            extra_acceleration_factor = (
                2  # Extra acceleration to simulate gravitational assistance
            )
            # Normalize the position vector
            r_vec_normalized = [r_vec[i] / r_mag for i in range(3)]
            # Apply the extra acceleration in the direction of the position vector
            vel = [
                vel[i] + r_vec_normalized[i] * extra_acceleration_factor * dt
                for i in range(3)
            ]

        return vel

    # Main simulation loop using the Leapfrog integrator
    while True:
        for planet_id in planets:
            pos = planet_data[planet_id]["pos"]
            vel = planet_data[planet_id]["vel"]

            # Calculate the current acceleration
            a = gravitational_acceleration(pos)

            # Update velocity at half step (v + 0.5 * a * dt)
            vel_half = [vel[i] + 0.5 * a[i] * dt for i in range(3)]

            # Full position update: pos_new = pos + vel_half * dt
            new_pos = [pos[i] + vel_half[i] * dt for i in range(3)]

            # Calculate acceleration with the new position
            a_new = gravitational_acceleration(new_pos)

            # Full velocity update: vel_new = vel_half + 0.5 * a_new * dt
            new_vel = [vel_half[i] + 0.5 * a_new[i] * dt for i in range(3)]

            # If the body is the asteroid, apply additional acceleration
            if planet_id == asteroid_id:
                new_vel = gravitational_assistance(new_pos, new_vel)

            # Store the new values in our data
            planet_data[planet_id]["pos"] = new_pos
            planet_data[planet_id]["vel"] = new_vel

            # Update the planet's position in the simulator
            p.resetBasePositionAndOrientation(planet_id, new_pos, [0, 0, 0, 1])

        # Camera controls
        keys = p.getKeyboardEvents()
        if ord("w") in keys and keys[ord("w")] & p.KEY_WAS_TRIGGERED:
            camera_pitch += 5
        if ord("s") in keys and keys[ord("s")] & p.KEY_WAS_TRIGGERED:
            camera_pitch -= 5
        if ord("a") in keys and keys[ord("a")] & p.KEY_WAS_TRIGGERED:
            camera_yaw -= 5
        if ord("d") in keys and keys[ord("d")] & p.KEY_WAS_TRIGGERED:
            camera_yaw += 5
        if ord("q") in keys and keys[ord("q")] & p.KEY_WAS_TRIGGERED:
            camera_distance += 1
        if ord("e") in keys and keys[ord("e")] & p.KEY_WAS_TRIGGERED:
            camera_distance -= 1

        # Update the camera
        update_camera()

        p.stepSimulation()
        time.sleep(dt)


if __name__ == "__main__":
    main()
