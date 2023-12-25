import math
import numpy as np
import matplotlib.pyplot as plt

# This program simulates the movement of an item in a vertical loop.

# Constants
g = 9.81  # gravitational acceleration in m/s^2
pi = math.pi

print("Vertical Loop Motion Simulation\n")

# Initialyze control flags
inp_flg = False  # Input data allow to continue working
exit_flg = False  # Permition to leave loop, even though input data under warning.
err_flg = False  # Inconsistency between loop radius and initial velocity prevent accomplishment of a complete loop

# Inputs
r = float(input("Enter the RADIUS of the loop (m): "))  # Ask loop radius
v0_min = math.sqrt(5 * r * g)

while not inp_flg and not exit_flg:
    v0 = float(input("Enter the INITIAL VELOCITY (m/s): "))  # Ask initial velocity
    if v0 < v0_min:
        print("\nWARNING: Initial velocity is too small (Limit =", v0_min,"m/s)\n")
        print("    C: Change velocity\n    G: Go on as is\n    S: Stop\n")
        resp = input(" => ")

        if resp == 'c' or resp == 'C':
            continue
        elif resp == 'g' or resp == 'G':
            inp_flg = True
        else:
            exit_flg = True
    else:
        inp_flg = True

# Continue working if a) input data is correct or b)the user asked to do it. 
if inp_flg:
    theta = math.pi * 3 / 2  # starting at the bottom of the loop
    h = 0  # start loop at height of reference ground

# Pre-calculate constants
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

# Simulation parameters
    dt = 0.001  # time step
    t_max = 10.0  # maximum simulation time

# Build and initialize time segments array
    t_data = np.arange(0, t_max + dt, dt)

# Build position and velocity arrays
    theta_data = np.zeros_like(t_data)  # Vertical angles array of each step
    x_data = np.zeros_like(t_data)  # X and Y arrays
    y_data = np.zeros_like(t_data)
    v_data = np.zeros_like(t_data)  # Total velocity and components
    vx_data = np.zeros_like(t_data)
    vy_data = np.zeros_like(t_data)

# Initialize energies at start point
    ep = 0  # Initial state (potential) energy
    ek = 0.5 * v0**2  # Initial kinetic) energy
    et = ep + ek  # Total mechanical energy

# Set initial conditions in arrays
    theta_data[0] = theta
    v_data[0] = v0
    x_data[0] = r * cos_theta
    y_data[0] = r * sin_theta
    vx_data[0] = -v0 * sin_theta
    vy_data[0] = v0 * cos_theta

# Loop simulation for each time tick
    for i in range(1, len(t_data)):

    # Calculate former angular velocity and calculate next angle
        omega = v_data[i - 1] / r
        theta = (theta + omega * dt) % (2 * pi)

    # Use shorts
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        # assign values to arrays
        theta_data[i] = theta
        x_data[i] = r * cos_theta
        y_data[i] = r * sin_theta

        # Calculate present kinetic energy (total energy fixed)
        h = r + y_data[i]
        ep = g * h
        ek = et - ep

# If iitial velocity was too small to climb along the complete loop (Energy considerations)
        if ek <= 0:
            print("\nERROR. A complete loop can't be accomplished.")
            print("\nItem slipped back down at angle: ", theta * 180 / pi,
            " degrees.\n")

            err_flg = True
            break

        # find velocity and components from kinetic energy
        v_data[i] = math.sqrt(2 * ek)
        vx_data[i] = -v_data[i] * sin_theta
        vy_data[i] = v_data[i] * cos_theta

        # No problem with kinetic energy. Is there a problem with centripital acceleration?
        if sin_theta >= 0:
            if v_data[i] <= math.sqrt(r * g * sin_theta):
                print("\nERROR. A complete loop can't be accomplished.")
                print("\nItem left the track at angle: ", theta * 180 / pi, " degrees.\n")
                err_flg = True
                break

if inp_flg and not err_flg:
# Velocity is big enough to make complete loop.
# Create a figure for visualization
    fig, ax = plt.subplots()

# Ask user how to presents the results
    graph = int(input("\nEnter type of graph\n 1: X / Y graph\n 2: Angle / X graph\n 3: Angle / Y graph\n 4: Time / V graph\n 5: Time / Vx graph\n 6: Time / Vy graph\n\n => "))

    if graph == 1:
        # Plot loop motion
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('X / Y graph')
        ax.plot(x_data, y_data)
        plt.show()
    elif graph == 2:
        # Angle / X graph
        ax.set_xlabel('Angle (rad)')
        ax.set_ylabel('X (m)')
        ax.set_title('Angle / X graph')
        ax.plot(theta_data, x_data)
        plt.show()
    elif graph == 3:
        # Angle / Y graph
        ax.set_xlabel('Angle (rad)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Angle / Y graph')
        ax.plot(theta_data, y_data)
        plt.show()
    elif graph == 4:
        # Time / V graph
        ax.set_xlabel('Time (sec)')
        ax.set_ylabel('V (m)')
        ax.set_title('Time / V graph')
        ax.plot(t_data, v_data)
        plt.show()
    elif graph == 5:
        # Time / Vx graph
        ax.set_xlabel('Time (sec)')
        ax.set_ylabel('Vx (m/sec)')
        ax.set_title('Time / Vx graph')
        ax.plot(t_data, vx_data)
        plt.show()
    elif graph == 6:
        # Time / Vy graph
        ax.set_xlabel('Time (sec)')
        ax.set_ylabel('Vy (m/sec)')
        ax.set_title('Time / Vy graph')
        ax.plot(t_data, vy_data)
        plt.show()
    else:
        print("Wrong Choice")