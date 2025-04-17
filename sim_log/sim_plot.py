import matplotlib.pyplot as plt

def plot(states):
    fig, axs = plt.subplots(3, 3, figsize=(12, 15))
    fig.suptitle("Simulation States")

    # Position x, y, z
    axs[0, 0].plot(states["time"], states["x"], label='x (m)', color='r')
    axs[0, 0].set_title("Position x")
    axs[0, 0].set_xlabel("Time (s)")
    axs[0, 0].set_ylabel("x (m)")
    axs[0, 0].legend()
    axs[0, 0].grid()

    axs[0, 1].plot(states["time"], states["y"], label='y (m)', color='g')
    axs[0, 1].set_title("Position y")
    axs[0, 1].set_xlabel("Time (s)")
    axs[0, 1].set_ylabel("y (m)")
    axs[0, 1].legend()
    axs[0, 1].grid()

    axs[0, 2].plot(states["time"], states["z"], label='z (m)', color='b')
    axs[0, 2].set_title("Position z")
    axs[0, 2].set_xlabel("Time (s)")
    axs[0, 2].set_ylabel("z (m)")
    axs[0, 2].legend()
    axs[0, 2].grid()

    # velocity u v w
    axs[1, 0].plot(states["time"], states["u"], label='u (m/s)', color='r')
    axs[1, 0].set_title("Position x")
    axs[1, 0].set_xlabel("Time (s)")
    axs[1, 0].set_ylabel("u (m/s)")
    axs[1, 0].legend()
    axs[1, 0].grid()

    axs[1, 1].plot(states["time"], states["v"], label='v (m/s)', color='g')
    axs[1, 1].set_title("Position y")
    axs[1, 1].set_xlabel("Time (s)")
    axs[1, 1].set_ylabel("v (m/s)")
    axs[1, 1].legend()
    axs[1, 1].grid()

    axs[1, 2].plot(states["time"], states["w"], label='w (m/s)', color='b')
    axs[1, 2].set_title("Position z")
    axs[1, 2].set_xlabel("Time (s)")
    axs[1, 2].set_ylabel("w (m/s)")
    axs[1, 2].legend()
    axs[1, 2].grid()

    # Angles (phi, theta)
    axs[2, 0].plot(states["time"], states["phi"], label='phi (rad)')
    axs[2, 0].plot(states["time"], states["theta"], label='theta (rad)')
    axs[2, 0].set_title("Angles (phi, theta)")
    axs[2, 0].set_xlabel("Time (s)")
    axs[2, 0].set_ylabel("Angle (rad)")
    axs[2, 0].legend()
    axs[2, 0].grid()

    # Angle (psi)
    axs[2, 1].plot(states["time"], states["psi"], label='psi (rad)')
    axs[2, 1].set_title("Heading angle (psi)")
    axs[2, 1].set_xlabel("Time (s)")
    axs[2, 1].set_ylabel("psi (rad)")
    axs[2, 1].legend()
    axs[2, 1].grid()

    # Angular Rates (p, q, r)
    axs[2, 2].plot(states["time"], states["p"], label='p (rad/s)')
    axs[2, 2].plot(states["time"], states["q"], label='q (rad/s)')
    axs[2, 2].plot(states["time"], states["r"], label='r (rad/s)')
    axs[2, 2].set_title("Angular Rates (p, q, r)")
    axs[2, 2].set_xlabel("Time (s)")
    axs[2, 2].set_ylabel("Angular Rate (rad/s)")
    axs[2, 2].legend()
    axs[2, 2].grid()

    plt.tight_layout()
    plt.savefig("sim_log/simulation.png")
    plt.show()
