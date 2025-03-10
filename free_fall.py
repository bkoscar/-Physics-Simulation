import pybullet as p
import pybullet_data
import random
import time


def main():
    """
    Simulates free-falling particles in a PyBullet environment.

    This function connects to the PyBullet simulator in GUI mode, sets up the simulation environment,
    creates particles with random initial positions and velocities, and runs the simulation until
    all particles have stopped moving.
    """
    # Connect to the simulator in GUI mode with full screen
    p.connect(p.GUI, options="--width=1920 --height=1080")

    # Set gravity (in m/s^2)
    p.setGravity(0, 0, -9.8)

    # Load the simulation plane
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Load PyBullet data
    planeId = p.loadURDF("plane.urdf")

    # Create particles (spheres) in random positions with initial velocity
    num_particles = 5  # Number of particles
    particles = []
    particle_radius = 0.5
    particle_mass = 5

    # Configure friction and restitution for the particles
    restitution = 1  # Coefficient of restitution (bounce)
    friction = 0.4  # Coefficient of friction

    # Create particles in random positions
    for _ in range(num_particles):
        # Random initial position of the particles within a defined range
        start_position = [
            random.uniform(-5, 5),
            random.uniform(-5, 5),
            random.uniform(5, 10),
        ]
        # Initial orientation (no rotation)
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])

        # Create the particle
        particleId = p.createMultiBody(
            particle_mass,
            p.createCollisionShape(p.GEOM_SPHERE, radius=particle_radius),
            basePosition=start_position,
            baseOrientation=start_orientation,
        )

        # Set dynamic properties (restitution, friction)
        p.changeDynamics(
            particleId,
            -1,
            restitution=restitution,
            lateralFriction=friction,
            spinningFriction=friction,
        )
        p.changeDynamics(planeId, -1, restitution=restitution, lateralFriction=friction)
        # Assign a random initial velocity to the particle (in 3D)
        initial_velocity = [
            random.uniform(-5, 5),
            random.uniform(-5, 5),
            random.uniform(-5, -1),
        ]
        p.resetBaseVelocity(particleId, linearVelocity=initial_velocity)

        # Store the particle ID
        particles.append(particleId)

    # Configure the camera for a better view
    p.resetDebugVisualizerCamera(
        cameraDistance=15, cameraYaw=90, cameraPitch=-30, cameraTargetPosition=[0, 0, 5]
    )

    # Simulation loop
    while True:
        p.stepSimulation()  # Execute a simulation step
        time.sleep(1.0 / 240.0)  # Control the simulation rate (240Hz)

        # Get the current positions and velocities of the particles
        for i, particleId in enumerate(particles):
            position, orientation = p.getBasePositionAndOrientation(particleId)
            velocity, _ = p.getBaseVelocity(
                particleId
            )  # Get only the velocity (without angular velocity)

        # Stop the simulation if all particles have touched the ground and stopped moving
        all_particles_stopped = True
        for particleId in particles:
            position, orientation = p.getBasePositionAndOrientation(particleId)
            velocity, _ = p.getBaseVelocity(particleId)
            if abs(velocity[2]) > 0.01:  # If the vertical velocity is significant
                all_particles_stopped = False
                break

        if all_particles_stopped:
            print("All particles have stopped moving.")
            time.sleep(5)  # Wait 5 seconds before closing the simulation
            break

    # Disconnect the simulation
    p.disconnect()


if __name__ == "__main__":
    main()
