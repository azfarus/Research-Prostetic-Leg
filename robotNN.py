import mujoco
from mujoco import viewer
import numpy as np
import time
from simple_pid import PID
import matplotlib.pyplot as plt
from autotune import MujocoAutotune
from nn_controller import NNController




# Load the MuJoCo model
model = mujoco.MjModel.from_xml_path("two_link_robot.xml")
data = mujoco.MjData(model)


view = viewer.launch_passive(model, data)  
start_time = time.time()

 
actual_trajectory = []
desired_trajectory = []
knee_accels = []
ankle_accels=[]
knee_vels=[]
ankle_vels=[]
timestep=0.01
total_sim_time= 20
current_time = 0


controller = NNController([50,30],5.25,.01,0.0001,timestep)


while True:
    
    
    setpoints = [ (np.pi/2)*np.sin(current_time), (np.pi/2)*np.cos(current_time)]
    setpoints_vel = [ (np.pi/2)*np.cos(current_time), -(np.pi/2)*np.sin(current_time)]
    setpoints_acc = [ -(np.pi/2)*np.sin(current_time),  (np.pi/2)*np.cos(current_time)]
    controller.setpointAnkle = [setpoints[0],setpoints_vel[0], setpoints_acc[0]]
    controller.setpointKnee = [setpoints[1],setpoints_vel[1], setpoints_acc[1]]
    
    



    
    knee_acc =  data.sensor('knee_acc').data[1]
    ankle_acc = data.sensor('ankle_acc').data[1]
    
    knee_vel = data.sensor('knee_vel').data[0]
    ankle_vel = data.sensor('ankle_vel').data[0]
    
    knee_pos = data.sensor('knee_pos').data[0]
    ankle_pos = data.sensor('ankle_pos').data[0]
    
    sensorVector = np.array(
        [knee_pos,knee_vel,knee_acc,ankle_pos,ankle_vel,ankle_acc]
    )
    
    val0 = controller.control(sensorVector)
    
    data.ctrl[0] = val0[0]
    data.ctrl[1] = val0[1]
    
    
    actual_trajectory.append([knee_pos,ankle_pos])
    desired_trajectory.append(setpoints)
    knee_vels.append( knee_vel)
    ankle_vels.append(ankle_vel)
    knee_accels.append(knee_acc)
    ankle_accels.append(ankle_acc)
    
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

ax[1,0].plot(knee_accels, color='r' )
ax[1,0].plot(ankle_accels, color='g')
ax[1,0].plot(np.array(ankle_accels) + np.array(knee_accels), color='b',linestyle="dotted")
ax[1,0].set_title("Accelerations")
ax[1,0].grid()

ax[1,1].plot(knee_vel, color='r' ,linestyle='--')
ax[1,1].plot(ankle_vel, color='g',linestyle='--')
ax[1,1].set_title("Velocities")
ax[1,1].grid()



plt.tight_layout()  # Adjust spacing
plt.show()
    
    
    
