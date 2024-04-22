import matplotlib.pyplot as plt
import numpy as np
import os

script_dir = os.path.dirname(__file__)

# Define the filename
motor_data = os.path.join(script_dir, 'plots/data/motor.csv')

transposed_data = np.loadtxt(motor_data, delimiter=',')

data = transposed_data.T


step = data[0]
pwm = data[1:3, :]
comp = data[3]
pos = data[4:6, :]
vel = np.diff(pos)/0.01
delta_pos = pos[0]-pos[1]



# Create subplots for each row (after transposing, rows become columns)
fig, axs = plt.subplots(4, 1, figsize=(8, 12))  # 6 rows, 1 column of subplots
fig.tight_layout(pad=3.0)  # Adjust spacing between subplots



axs[0].plot(step, pwm[0])
axs[0].plot(step, pwm[1])
axs[0].set_title('Motor Plots - No resistance, Battery Power\nPWM Signal vs Time')

axs[1].plot(step[:-1], vel[0])
axs[1].plot(step[:-1], vel[1])
axs[1].set_title('Left and Right Velocities vs Time')

axs[2].plot(step, comp)
axs[2].set_title('Compensation factor vs Time')

axs[3].plot(step, delta_pos)
axs[3].set_title('Difference between left and right encoders')


plt.show()