import numpy as np


class NNController:
    def __init__(self,kv: np.ndarray,lambd: float , kapp: float, robust: float, timestep: float):
        
        self.setpointKnee: np.ndarray = np.zeros(3)
        self.setpointAnkle: np.ndarray = np.zeros(3)
        
        self.kv :np.ndarray = kv
        self.lambd : float = lambd
        self.kapp : float = kapp
        self.robust : float = robust
        self.learning_rate=.001
        self.timestep = timestep
        self.prev_e = np.array([0,0])
        
        self.weights = np.random.random((10,2))
        
        self.centers = np.linspace(0 , 0 , 10)
        self.sigma = np.linspace(np.pi,2*np.pi , 10)

    
    def control(self, sensorVector: np.ndarray)-> np.ndarray:
        
        measurementKnee = sensorVector[0:3]
        measurementAnkle = sensorVector[3:6]
        
        e = -np.array([
            measurementKnee[0] - self.setpointKnee[0],
            measurementAnkle[0] - self.setpointAnkle[0]
        ])
        e_dot = (e - self.prev_e) / self.timestep
        self.prev_e = e
        
        r = e_dot + self.lambd * e
        f_hat = self.train(e,e_dot,r,sensorVector)
        u = self.robust * self.signum(r)
        
        tau = f_hat + self.kv * r - u
        
        return tau[0]
    
    def train(self, e,e_dot,r,sensorVector):
        x = np.concatenate((e,e_dot,sensorVector),axis=0)
        
        h = np.array([self.rbf(x,self.centers,self.sigma[i]) for i in range(10)])
        
        h = np.reshape(h,(10,1))
        r = np.reshape(r,(2,1))
        
        # print(h)
        f_hat = (self.weights.T @ h ).T
        w_dot = h @ r.T - self.kapp * np.linalg.norm(r) * self.weights
        self.weights -= self.learning_rate*w_dot
                
        return np.clip(f_hat,-10000,10000)
        
    
    def signum(self,x:float):
        return x/abs(x)
    
    def rbf(self,x , c , s):
        return np.exp( -(x-c)/(2*(s**2))).sum()
    