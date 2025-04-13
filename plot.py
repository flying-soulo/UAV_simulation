import pandas as pd
import matplotlib.pyplot as plt

def plot_uav_simulation(file_path):
    # Load the data
    df = pd.read_csv(file_path)

    time = df['time']

    # Create figure and subplots
    fig, axs = plt.subplots(4, 1, figsize=(12, 16), sharex=True)
    fig.suptitle('UAV Simulation Overview', fontsize=16)

    # 1. Longitudinal Dynamics
    axs[0].plot(time, df['x'], label='x (forward)')
    axs[0].plot(time, df['z'], label='z (altitude)')
    axs[0].plot(time, df['u'], label='u (vx)')
    axs[0].plot(time, df['w'], label='w (vz)')
    axs[0].plot(time, df['theta'], label='theta (pitch angle)')
    axs[0].plot(time, df['q'], label='q (pitch rate)')
    axs[0].set_title('Longitudinal Dynamics')
    axs[0].legend()
    axs[0].grid(True)

    # 2. Lateral-Directional Dynamics
    axs[1].plot(time, df['y'], label='y (lateral)')
    axs[1].plot(time, df['v'], label='v (vy)')
    axs[1].plot(time, df['phi'], label='phi (roll angle)')
    axs[1].plot(time, df['psi'], label='psi (yaw angle)')
    axs[1].plot(time, df['p'], label='p (roll rate)')
    axs[1].plot(time, df['r'], label='r (yaw rate)')
    axs[1].set_title('Lateral-Directional Dynamics')
    axs[1].legend()
    axs[1].grid(True)

    # 3. Forces
    axs[2].plot(time, df['Fx'], label='Fx (forward force)')
    axs[2].plot(time, df['Fy'], label='Fy (side force)')
    axs[2].plot(time, df['Fz'], label='Fz (vertical force)')
    axs[2].set_title('External Forces')
    axs[2].legend()
    axs[2].grid(True)

    # 4. Moments
    axs[3].plot(time, df['l'], label='l (roll moment)')
    axs[3].plot(time, df['m'], label='m (pitch moment)')
    axs[3].plot(time, df['n'], label='n (yaw moment)')
    axs[3].set_title('Moments')
    axs[3].legend()
    axs[3].set_xlabel('Time (s)')
    axs[3].grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.97])
    plt.show()

if __name__ == "__main__":
    plot_uav_simulation("simulation.csv")