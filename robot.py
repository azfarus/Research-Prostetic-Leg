import mujoco
from mujoco import viewer
import numpy as np
import time
from simple_pid import PID
import matplotlib.pyplot as plt
from autotune import MujocoAutotune





# Load the MuJoCo model
model = mujoco.MjModel.from_xml_path("two_link_robot.xml")
data = mujoco.MjData(model)


view = viewer.launch_passive(model, data)  
start_time = time.time()

controller1 = PID(14000, 0, 100)
controller2 = PID(1200, 0, 120)
 
actual_trajectory = []
desired_trajectory = []
knee_accel = []
ankle_accel=[]
knee_vels=[]
ankle_vels=[]
timestep=0.01
total_sim_time= 10
current_time = 0

while True:
    
    
    setpoints = [ (np.pi/2)*np.sin(current_time), (np.pi/2)*np.cos(current_time)]
    controller1.setpoint = setpoints[0]
    controller2.setpoint = setpoints[1]
    
    val0 = controller1(data.sensor('knee_pos').data[0])
    val1 = controller2(data.sensor('ankle_pos').data[0])
    
    data.ctrl[0] = val0
    data.ctrl[1] = val1

    actual_trajectory.append(data.sensordata.copy())
    desired_trajectory.append(setpoints)

    
    knee_acc = data.sensor('knee_acc').data[1]
    ankle_acc = data.sensor('ankle_acc').data[1]
    
    knee_vel = data.sensor('knee_vel').data[0]
    ankle_vel = data.sensor('ankle_vel').data[0]
    
    knee_pos = data.sensor('knee_pos').data[0]
    ankle_pos = data.sensor('ankle_pos').data[0]
    
    sensorVector = np.array(
        [knee_pos,knee_vel,knee_acc,ankle_pos,ankle_vel,ankle_acc]
    )
    
    
    
    knee_vels.append( knee_vel)
    ankle_vels.append(ankle_vel)
    
    mujoco.mj_step(model, data)
    time.sleep(timestep)
    view.sync()
    
    if(current_time >= total_sim_time):
        break
    current_time += timestep
    

actual_trajectory=np.array(actual_trajectory, dtype=np.float32)
desired_trajectory=np.array(desired_trajectory,dtype=np.float32)

fig, ax = plt.subplots(2, 2, figsize=(10, 4))  # 1 row, 2 columns

ax[0,0].plot(actual_trajectory[:,0], color='r')
ax[0,0].plot(desired_trajectory[:,0], color='g')
ax[0,0].set_title("Knee_joint")
ax[0,0].grid()

ax[0,1].plot(actual_trajectory[:,1], color='r' ,linestyle='--')
ax[0,1].plot(desired_trajectory[:,1], color='g',linestyle='--')
ax[0,1].set_title("ankle_joint")
ax[0,1].grid()

ax[1,0].plot(knee_accel, color='r' )
ax[1,0].plot(ankle_accel, color='g')
ax[1,0].plot(np.array(ankle_accel) + np.array(knee_accel), color='b',linestyle="dotted")
ax[1,0].set_title("Accelerations")
ax[1,0].grid()

ax[1,1].plot(knee_vel, color='r' ,linestyle='--')
ax[1,1].plot(ankle_vel, color='g',linestyle='--')
ax[1,1].set_title("Velocities")
ax[1,1].grid()



plt.tight_layout()  # Adjust spacing
plt.show()
    
    
    
