import mujoco
from scipy import optimize
import numpy as np  
from simple_pid import PID
from mujoco import viewer
import time

class MujocoAutotune:
   
    def __init__(self, model, data):
        self.model = model
        self.data = data

        
    def autotune(self):
        result = optimize.minimize(run_sim, [12000.0,0.0,400.0], args=(self.model, self.data))
        print(result)
        return result
        



def run_sim(k, model , data):
    timestep = 0.01
    controller = PID(k[0], k[1], k[2])

    actual_trajectory = []
    desired_trajectory = []
    controller.sample_time = timestep
    current_time=0
    
    
    while True:
        
        
        setpoints = [ (np.pi/2)*np.sin(current_time), (np.pi/2)*np.cos(current_time)]
        controller.setpoint = setpoints
        
        val = controller(data.sensordata)
        
        data.ctrl[0] = val[0]
        data.ctrl[1] = val[1]

        actual_trajectory.append(data.sensordata.copy())
        desired_trajectory.append(setpoints)

        mujoco.mj_step(model, data)        
        if(np.floor(current_time) >= 10):
            break     
        current_time += timestep
        time.sleep(timestep)
        
    actual_trajectory = np.array(actual_trajectory)
    desired_trajectory = np.array(desired_trajectory)
    diff = actual_trajectory - desired_trajectory
    error = np.mean(np.square(diff),dtype=np.float64)
    # error_abs = np.mean(np.abs(diff),dtype=np.float64)
    print(error)
    return error



