from abc import ABCMeta
from abc import abstractmethod
import numpy as np
from scipy.integrate import solve_ivp

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from functools import partial

class Model:
    __metaclass__ = ABCMeta
    
    def __init__(self, state: np.array, dt: float) -> None:
        self._state = state     # vehicle state
        self._dt = dt           # time step
        
    @abstractmethod
    def step(self, u: np.array):
        pass
    
    @property
    def n(self):
        return len(self.state)
    
    @property
    @abstractmethod
    def m(self):
        pass
    
    @property
    def state(self):
        return self._state
    
    @state.setter
    def state(self, state):
        self._state = state   
        
class UnicycleModel(Model):
    def __init__(self, state: np.array, dt: float, Iw=0.000125, If=0.01125, mw=0.1, mf=1.0, Rw=0.05, L=0.15, e=0.1, delta=0.05) -> None:
        # state is vehicle state: x, y, theta
        super().__init__(state, dt)
        self._Iw = Iw
        self._If = If
        self._Ip = self._If + 2*self._Iw
        
        self._mw = mw
        self._mf = mf
        self._mp = self._mf + 2*self._mw
        
        self._Rw = Rw
        self._L = L
        
        self._e = e
        self._delta = delta
            
    def step(self, u: np.array):
        K = np.array([[np.cos(self.state[2]), 0],
                      [np.sin(self.state[2]), 0],
                      [0                    , 1]])
        dx = np.matmul(K, u)    # matrix multiplication
        self._state = self._state + dx * self._dt
        
        return self._state
        
    @property
    def m(self):
        return 2
    
    @property
    def M(self):
        mp = self._mp
        Ip = self._Ip
        Iw = self._Iw
        M = np.array([[mp, 0, 0, 0, 0],
                    [0, mp, 0, 0, 0],
                    [0, 0, Ip, 0, 0],
                    [0, 0, 0, Iw, 0],
                    [0, 0, 0, 0, Iw],])
        return M
    
    @property
    def B(self):
        B = np.array([[0,0],
                      [0,0],
                      [0,0],
                      [1,0],
                      [0,1],])
        return B
    
    @property
    def G(self):
        q = self._state
        L = self._L
        Rw = self._Rw
        G = np.array([[np.cos(q[2]), np.cos(q[2])],
                      [np.sin(q[2]), np.sin(q[2])],
                      [1/L,-1/L],
                      [2/Rw,0],
                      [0,2/Rw],])
        return G
    
    def G_d1(self, state_d1):
        q = self._state
        q_d1 = state_d1
        G_d1 = np.array([[-np.sin(q[2]) * q_d1[2], -np.sin(q[2]) * q_d1[2]],
                         [ np.cos(q[2]) * q_d1[2],  np.cos(q[2]) * q_d1[2]],
                         [0,0],
                         [0,0],
                         [0,0],])   
        return G_d1    
 
    
    @property
    def e(self):
        return self._e
    
    @property
    def delta(self):
        return self._delta
    
    @property
    def h(self):
        e = self._e
        delta = self._delta
        x = self._state[0]
        y = self._state[1]
        theta = self._state[2]
        h = np.array([x + e * np.cos(theta + delta),
                      y + e * np.sin(theta + delta),])
        return h       
        

def trajectory_generator_square(t, dt=1):
    # TODO: generate square trajectory
    # with e.g. piece wise approach
    if type(t) != np.ndarray:
        L = 1  # length of side of a square
        v = L / dt  # constant velocity; dt -- time needed to do one side
        phase = int(t % (4 * dt))    # 4dt -- time to do the whole square once
        position = v * (t % (4 * dt) % dt)  # ile procent trasy na danym boku udało się już przejść w danym t

        if phase == 0:          # move to the right from [0,0] to [L, 0]
            h = np.array([position, 0])  
            h_d1 = np.array([v, 0])
        elif phase == 1:        # move up from [L,0] to [L,L]
            h = np.array([L, position])
            h_d1 = np.array([0, v])
        elif phase == 2:        # move left from [L,L] to [0,L]
            h = np.array([L - position, L])
            h_d1 = np.array([-v, 0])
        else:                   # move down from [0,L] to [0,0]
            h = np.array([0, L - position])
            h_d1 = np.array([0, -v])

        h_d2 = np.array([0, 0])  # constant velocity, no acceleration


    else:
        # do not change below code of this function
        h = np.zeros((2, len(t)))
        h_d1 = np.zeros((2, len(t)))
        h_d2 = np.zeros((2, len(t)))
        for i in range(len(t)):
            h[:,i], h_d1[:,i], h_d2[:,i] = trajectory_generator_square(t[i])
        
    return h, h_d1, h_d2


def trajectory_generator_circle(t, w=np.pi * 0.4, offset=0.2, A=1.0):
    h = np.array([A*np.cos(t*w + offset), A*np.sin(t*w + offset)])
    # TODO: calculate first and second derivative
    h_d1 = np.array([-A * w * np.sin(t * w + offset), A * w * np.cos(t * w + offset)])
    h_d2 = np.array([-A * w**2 * np.cos(t * w + offset), -A * w**2 * np.sin(t * w + offset)])
    return h, h_d1, h_d2


class Simulator:
    __metaclass__ = ABCMeta
        
    def __init__(self, model, dt=0.01) -> None:
        self._model = model
        self._stats = {}
        self._dt = dt
        
    @abstractmethod
    def step(self, t, state):
        pass
        
    def run(self, start: np.array, T: float, dt: float, trajectory):
        self.T = T
        self.dt = dt
        self._trajectory = trajectory
        
        startTime = 0
        self._stats = {'t': [startTime], 
                 'next': 0}
        
        solver = solve_ivp(self.step, [0, T], start, method='RK45', 
                           rtol=1e-3, atol=1e-6, t_eval=list(np.arange(startTime,T+self._dt,self._dt)))
        
        return self._stats, solver
    
    
class SimulatorDynamics(Simulator):
    def __init__(self, model, dt=0.01) -> None:
        self._model = model
        self._stats = {}
        self._dt = dt
        
    def step(self, t, state):
        h = state[0:2]
        h_d1 = state[2:4]
        k = state[4:9]
        q = k.reshape((-1,))
        
        e = self._model.e
        delta = self._model.delta
        
        self._model.state = q
        M = self._model.M
        B = self._model.B
        G = self._model.G
        
        # TODO: calculate partial derivative for dh/dq
        # h(q) diffeomorphism function was given during the 
        # lecture. It can be found also in Unicycle.h() method 

        theta = q[2]
        dh_dq = np.array([[1, 0, -e * np.sin(theta + delta)],
                          [0, 1, e * np.cos(theta + delta)]])
        Rinv = dh_dq @ G[0:3,:]
        detRinv = np.linalg.det(Rinv)
        
        R = np.linalg.inv(Rinv)
        detR = np.linalg.det(R)
        RT = R.T
        
        # TODO: Calculate k_d1, k' which will 
        # reflect system velocities q'
        k_d1 = G @ R @ h_d1     # @ is to multiply matrices
        
        q_d1 = k_d1.reshape((-1,))
        
        e = self._model.e
        delta = self._model.delta
        
        R_d1 = q_d1[2] / np.cos(delta) * \
            np.array([[- np.sin(q[2] + delta),   np.cos(q[2] + delta)],
                      [- np.cos(q[2]) / e, - np.sin(q[2]) / e]])

        G_d1 = self._model.G_d1(q_d1)
                
        GT = G.T
        
        # TODO: calculate Ms, Cs and Bs
        # these matrices represent Unicycle's dynamics 
        # expressed in auxiliary velocities
        # use lecture notes. Remember to use 
        # np.dot() or '@' to multiply matrices together
        Ms = GT @ M @ G
        Cs = GT @ M @ G_d1
        Bs = GT @ B
        
        # TODO: calculate Mh, Ch and Bh
        # these matrices represent Unicycle's dynamics 
        # expressed in linearized coordinates
        # use lecture notes. Remember to use 
        # np.dot() or '@' to multiply matrices together        
        Mh = RT @ Ms @ R
        Ch = RT @ (Ms @ R_d1 + Cs @ R)
        Bh = RT @ Bs
        
        hd, hd_d1, hd_d2 = self._trajectory(t)
        
        Mhinv = np.linalg.inv(Mh)
        Bhinv = np.linalg.inv(Bh)
        
        Kp = 200
        Kd = 20
        
        # TODO: calculate errors and theirs first derivative
        eh = h - hd
        eh_d1 = h_d1 - hd_d1
        
        # TODO: introduce new input to the system
        v = hd_d2 - Kp * eh - Kd * eh_d1
             
        # TODO: calculate control signals       
        Dh = np.zeros((2))
        Fh = -Mhinv @ Ch @ h_d1 - Mhinv @ Dh
        Gh = Mhinv @ Bh
        
        u = np.linalg.inv(Gh) @ (v - Fh)
        
        # TODO calculate h second derivative, h''(q)
        h_d2 = Fh + Gh @ u
        
        new_state = np.concatenate([h_d1, h_d2, k_d1])
        
        if t >= self._stats['next']:
            self._stats['next'] = t + self._dt
            print(f"t: {t:.2f}, "
                    f"e_h: {eh}, h: {h}, hd: {hd}, "
                    f"x: {q[0]:.2f}, y: {q[1]:.2f}, theta: {q[2]:.2f}, "
                    f"detRinv: {detRinv:.6f}, detR: {detR:.6f}")
        
        return np.array(new_state)


class SimulatorKinematics(Simulator):   
    def __init__(self, model, dt=0.01) -> None:
        self._model = model
        self._stats = {}
        self._dt = dt
        
    def step(self, t, state):
        h = state[0:2]
        _ = state[2:4]
        k = state[4:9]
        q = k.reshape((-1,))
        
        e = self._model.e
        delta = self._model.delta
        
        self._model.state = q
        G = self._model.G

        theta = q[2]
        dh_dq = np.array([[1, 0, -e * np.sin(theta + delta)],
                          [0, 1, e * np.cos(theta + delta)]])
        Rinv = dh_dq @ G[0:3,:]
        detRinv = np.linalg.det(Rinv)
        
        R = np.linalg.inv(Rinv)
        detR = np.linalg.det(R)
        
        hd, hd_d1, _ = self._trajectory(t)
        
        # TODO: some calculations
        eh = h - hd 
        Kp = 200
        h_d1 = hd_d1 - Kp * eh
        k_d1 = G @ R @ h_d1
        
        h_d2 = np.array([0, 0])        
        new_state = np.concatenate([h_d1, h_d2, k_d1])
        
        if t >= self._stats['next']:
            self._stats['next'] = t + self._dt
            print(f"t: {t:.2f}, "
                    f"e_h: {eh}, h: {h}, hd: {hd}, "
                    f"x: {q[0]:.2f}, y: {q[1]:.2f}, theta: {q[2]:.2f}, "
                    f"detRinv: {detRinv:.6f}, detR: {detR:.6f}")
        
        return np.array(new_state)