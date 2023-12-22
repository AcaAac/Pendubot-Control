import scipy.io
import matplotlib.pyplot as plt
import numpy as np
import os

# Set rcParams to customize tick labels and spines
plt.rcParams['xtick.labelsize'] = 15
plt.rcParams['ytick.labelsize'] = 15
plt.rcParams['axes.labelsize'] = 15
plt.rcParams['axes.spines.top'] = False
plt.rcParams['axes.spines.right'] = False
plt.rcParams['axes.grid'] = True
plt.rcParams['legend.fontsize'] = 15
plt.rcParams['legend.loc'] = 'upper right'
plt.rcParams['axes.titlesize'] = 18

# PLOT LINEAR MODEL VALIDATION
path = r'linearization\ok_version\matlab.mat'
ttime = np.array(scipy.io.loadmat(path)['time']).reshape(1144)
data = [scipy.io.loadmat(path)['linearized_around_0_y'], scipy.io.loadmat(path)['discrete_linearized_around_0_y1'], scipy.io.loadmat(path)['simulator_x']]

fig1, ax1 = plt.subplots(nrows=2, ncols=2, figsize=(15, 9))
titles = [[r'$\alpha_1$', r'$\alpha_2$'],[r'$\dot{\alpha}_1$', r'$\dot{\alpha}_2$']]
y_label = ['angle [rad]', 'angular speed [rad/s]']
for i in range(2):
    for j in range(2):
        ax1[i][j].plot(ttime, (data[2].T)[i*2+j], '-', color='k', label='simulator')
        ax1[i][j].plot(ttime, (data[0].T)[i*2+j], '--', color='r', label='continuous li')
        ax1[i][j].legend()
        ax1[i][j].set_title(titles[i][j])
        if i == 1:
            ax1[i][j].set_xlabel('time [s]')
        if j == 0:
            ax1[i][j].set_ylabel(y_label[i])
fig2, ax2 = plt.subplots(nrows=2, ncols=2, figsize=(15, 9))
for i in range(2):
    for j in range(2):
        ax2[i][j].plot(ttime, (data[2].T)[i*2+j], '-', color='k', label='simulator')
        ax2[i][j].plot(ttime, (data[1].T)[i*2+j], '--', color='r', label='discrete li')
        ax2[i][j].legend()
        ax2[i][j].set_title(titles[i][j])
        if i == 1:
            ax2[i][j].set_xlabel('time [s]')
        if j == 0:
            ax2[i][j].set_ylabel(y_label[i])

# PLOT C1 sim VALIDATION
paths = [r'c3\matlab.mat']

for path in paths:
    tttime = np.array(scipy.io.loadmat(path)['time'])
    tttime.reshape(len(tttime))
    data = np.array(scipy.io.loadmat(path)['sim_C3'])

    figg, axx = plt.subplots(nrows=1, ncols=1, figsize=(15, 9))

    y_label = ['angle [rad]', 'angular speed [rad/s]']
    # axx.plot(tttime, np.ones(len(tttime))*np.pi*4/5, color='k', label=r'setpoint $\alpha_1$')
    axx.plot(tttime, np.ones(len(tttime))*np.pi, '-', color='k', label=r'setpoint')
    axx.plot(tttime, data[:,0]+np.pi, color='r', label=r'$\alpha_1$')
    axx.plot(tttime, -data[:,1]-np.pi, color='b', label=r'$\alpha_2$')
    axx.legend()
    axx.set_xlabel('time [s]')
    axx.set_ylabel('angle [rad]')

    # Split the file path into root and extension
    root, extension = os.path.splitext(path)

    # Replace the extension with ".pdf"
    pdf_path = f'{root}.pdf'


    figg.savefig(pdf_path, format="pdf", bbox_inches='tight')

# PLOT REAL C1

# Specify the file path
file_paths = [r'test_real_c1\test_C1_1.txt', 
              r'test_real_c1\test_C1_2.txt', 
              r'test_real_c1\test_C1_3.txt',
              r'test_real_c1\test_C1_4.txt',
              r'test_real_c1\test_C1_5.txt', 
              r'test_real_c1\test_C1_6.txt',
              r'test_real_c1\test_C1_7.txt',
              r'test_real_c1\test_C1_8.txt',
              r'test_real_c1\test_C1_9.txt',
              r'test_real_c1\test_C1_10.txt',
              r'test_real_c1\test_C1_final.txt',
              r'test_OL\openloop_0_1.txt']

a2_r = np.array([np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.pi, np.nan])
t1 = [60, 60, 60, 60, 60, 60, 60, 0, 0, 0, 0, 0]
t2 = [1000, 1000, 1000, 1000, 1000, 1000, 1000, int(5/0.004), 5001, 5001, 5001, 5001]
# Define the array size
rows, cols = 5001, 8

# Initialize the array
data_array = np.zeros((rows, cols))

# Open the file and read lines
for i, file_path in enumerate(file_paths):
    with open(file_path, 'r') as file:
        # Skip the first three lines
        for _ in range(3):
            next(file)

        # Read and store data into the array
        for row_num, line in enumerate(file):
            # Split the line into elements (assuming elements are separated by spaces)
            elements = line.split()

            # Store elements in the array
            for col_num, element in enumerate(elements):
                # Replace comma with dot (for non-integer elements)
                element = element.replace(',', '.')
                
                # Store elements in the array
                data_array[row_num, col_num] = float(element)

    # Extract time and columns for plotting
    time_column = np.array(data_array[:, 0]) * 0.004
    a1_r = data_array[:, 2]
    a1 = data_array[:, 3]
    da1 = data_array[:, 4]
    a2 = data_array[:, 5]
    da2 = data_array[:, 6]

    # Plot the data
    fig3, ax3 = plt.subplots(nrows=1, ncols=1, figsize=(15, 9))
    ax3.plot(time_column[t1[i]:t2[i]], a1_r[t1[i]:t2[i]], color='k', label=r'setpoint $\alpha_1$')
    if not np.isnan(a2_r[i]):
        ax3.plot(time_column[t1[i]:t2[i]], np.ones(t2[i]-t1[i])*a2_r[i], '--', color='k', label=r'setpoint $\alpha_2$')
    if np.isnan(a2_r[i]):
        ax1[0][0].plot(ttime, a1[:len(ttime)], '-.', color='b', label='real')
        ax1[0][1].plot(ttime, a2[:len(ttime)], '-.', color='b', label='real')
        ax1[1][0].plot(ttime, da1[:len(ttime)], '-.', color='b', label='real')
        ax1[1][1].plot(ttime, da2[:len(ttime)], '-.', color='b', label='real')
        ax2[0][0].plot(ttime, a1[:len(ttime)], '-.', color='b', label='real')
        ax2[0][1].plot(ttime, a2[:len(ttime)], '-.', color='b', label='real')
        ax2[1][0].plot(ttime, da1[:len(ttime)], '-.', color='b', label='real')
        ax2[1][1].plot(ttime, da2[:len(ttime)], '-.', color='b', label='real')
        ax1[0][0].legend()
        ax1[0][1].legend()
        ax1[1][0].legend()
        ax1[1][1].legend()
        ax2[0][0].legend()
        ax2[0][1].legend()
        ax2[1][0].legend()
        ax2[1][1].legend()
        

    ax3.plot(time_column[t1[i]:t2[i]], a1[t1[i]:t2[i]], color='r', label=r'$\alpha_1$')
    ax3.plot(time_column[t1[i]:t2[i]], a2[t1[i]:t2[i]], color='b', label=r'$\alpha_2$')

    # Add labels and legend
    ax3.set_xlabel('time [s]')
    ax3.set_ylabel('angle [rad]')
    ax3.legend()

    # Split the file path into root and extension
    root, extension = os.path.splitext(file_path)

    # Replace the extension with ".pdf"
    pdf_path = f'{root}.pdf'


    fig3.savefig(pdf_path, format="pdf", bbox_inches='tight')

fig1.savefig(r"plot\validate_li_cont.pdf", format="pdf", bbox_inches='tight')
fig2.savefig(r"plot\validate_li_discr.pdf", format="pdf", bbox_inches='tight')
