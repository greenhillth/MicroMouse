import matplotlib.pyplot as plt
import numpy as np
import os

script_dir = os.path.dirname(__file__)

# Define the filename
motor_data = os.path.join(script_dir, 'plots/data/move.csv')

data = np.genfromtxt(motor_data, delimiter=',', names=True, dtype=float)

step = data['step']
pwm_l = data['pwm_l']
pwm_r = data['pwm_r']
comp = data['comp_sig']
pos_l = data['enc_l']
pos_r = data['enc_r']
sp_l = data['target_left']
sp_r = data['target_right']

vel_l = np.diff(pos_l)/0.01
vel_r = np.diff(pos_r)/0.01
delta_pos = pos_l - pos_r



# Create subplots for each row (after transposing, rows become columns)
fig, axs = plt.subplots(4, 1, figsize=(8, 12))  # 6 rows, 1 column of subplots
fig.tight_layout(pad=3.0)  # Adjust spacing between subplots



axs[0].plot(step, pwm_l, label='Left PWM')
axs[0].plot(step, pwm_r, label='Right PWM')
axs[0].set_title('Motor PWM Plots\nPWM Signal vs Time')
axs[0].legend()

axs[1].plot(step, pos_l, label='Left Encoder Position')
axs[1].plot(step, pos_r, label='Right Encoder Position')
axs[1].plot(step, sp_l, label='Left Encoder Setpoint')
axs[1].plot(step, sp_r, label='Right Encoder Setpoint')
axs[1].set_title('Left and Right Encoder Positions vs Time')
axs[1].legend()

axs[2].plot(step[:-1], vel_l, label='Left Velocity')
axs[2].plot(step[:-1], vel_r, label='Right Velocity')
axs[2].set_title('Left and Right Velocities vs Time')
axs[2].legend()

axs[3].plot(step, delta_pos, label='Delta L-R')
axs[3].plot(step, comp, label='Compensation')
axs[3].set_title('Difference between left and right encoders')
axs[3].legend()


plt.show()