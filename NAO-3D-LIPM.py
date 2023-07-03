import numpy as np
import time
import math as m
from zmqRemoteApi import RemoteAPIClient
client = RemoteAPIClient()
sim = client.getObject('sim')
sim.startSimulation()

COM = sim.getObject('/NAO')
legr = sim.getObjectHandle('/NAO/Rjoint')
Rl2 = sim.getObjectHandle('/NAO/Rl2')
Rl3 = sim.getObjectHandle('/NAO/Rl3')
Rl4 = sim.getObjectHandle('/NAO/Rl4')
Rl5 = sim.getObjectHandle('/NAO/Rl5')
Rl6 = sim.getObjectHandle('/NAO/Rl6')
legl = sim.getObjectHandle('/NAO/Ljoint')
Ll2 = sim.getObjectHandle('/NAO/Ll2')
Ll3 = sim.getObjectHandle('/NAO/Ll3')
Ll4 = sim.getObjectHandle('/NAO/Ll4')
Ll5 = sim.getObjectHandle('/NAO/Ll5')
Ll6 = sim.getObjectHandle('/NAO/Ll6')
joint_handles = [legl,Rl2, Rl3, Rl4, Rl5, Rl6, legr, Ll2, Ll3, Ll4, Ll5, Ll6]
print(legr)
print(legl)


class LIPM3D():
    def __init__(self, dt=0.001, T_sup=1.0, support_leg='left_leg'):
        self.dt = dt
        self.t = 0
        self.T_sup = T_sup

        self.p_x = 0  # desired foot location x
        self.p_y = 0  # desired foot location y

        self.p_x_star = 0 # modified foot location x
        self.p_y_star = 0 # modified foot location y

        # Initialize the gait parameters
        self.s_x = 0.0
        self.s_y = 0.0

        # COM initial state
        self.x_0 = 0
        self.vx_0 = 0
        self.y_0 = 0
        self.vy_0 = 0

        # COM real-time state
        self.x_t = 0
        self.vx_t = 0
        self.y_t = 0
        self.vy_t = 0

        # COM desired state
        self.x_d = 0
        self.vx_d = 0
        self.y_d = 0
        self.vy_d = 0

        # final state for one gait unit
        self.bar_x = 0.0
        self.bar_y = 0.0
        self.bar_vx = 0.0
        self.bar_vy = 0.0

        self.support_leg = support_leg
        self.left_foot_pos = [0.0, 0.0, 0.0]
        self.right_foot_pos = [0.0, 0.0, 0.0]
        self.COM_pos = [0.0, 0.0, 0.0]

    def initializeModel(self, COM_pos, x_left_foot, y_left_foot, z_left_foot, x_right_foot, y_right_foot, z_right_foot):
        self.left_foot_pos = [x_left_foot, y_left_foot, z_left_foot]
        self.right_foot_pos = [x_right_foot, y_right_foot, z_right_foot]

        self.COM_pos = [(self.left_foot_pos[0] + self.right_foot_pos[0]) / 2, (self.left_foot_pos[1] + self.right_foot_pos[1]) / 2,(self.left_foot_pos[2] + self.right_foot_pos[2]) / 2]

        self.zc = self.COM_pos[2]
        self.T_c = np.sqrt(self.zc / 9.81)  # Establecer el parámetro de gravedad en 9.81
        self.C = np.cosh(self.T_sup / self.T_c)
        self.S = np.sinh(self.T_sup / self.T_c)

    def updateParameters(self, T_sup):
        self.T_sup = T_sup
        self.C = np.cosh(self.T_sup/self.T_c)
        self.S = np.sinh(self.T_sup/self.T_c)
    def step(self):
        # Calculate the desired state for the next time step
        self.x_d = self.p_x + self.s_x
        self.vx_d = (self.x_d - self.x_t) / self.dt
        self.y_d = self.p_y + self.s_y
        self.vy_d = (self.y_d - self.y_t) / self.dt

        # Calculate the final state for one gait unit
        self.bar_x = self.x_d + self.C * (self.vx_d * self.T_c - self.vx_0) - self.vx_d * self.T_c
        self.bar_vx = (self.bar_x - self.x_0) / self.T_sup
        self.bar_y = self.y_d + self.C * (self.vy_d * self.T_c - self.vy_0) - self.vy_d * self.T_c
        self.bar_vy = (self.bar_y - self.y_0) / self.T_sup

        # Calculate the real-time state using the final state for one gait unit
        self.x_t = self.x_0 + self.bar_vx * self.t
        self.vx_t = self.bar_vx
        self.y_t = self.y_0 + self.bar_vy * self.t
        self.vy_t = self.bar_vy

        # Update the support leg
        if self.t >= self.T_sup:
            if self.support_leg == 'left_leg':
                self.support_leg = 'right_leg'
            else:
                self.support_leg = 'left_leg'

        # Update the time
        self.t += self.dt

    def calculateXtVt(self, t):
        T_c = self.T_c

        x_t = self.x_0*np.cosh(t/T_c) + T_c*self.vx_0*np.sinh(t/T_c)
        vx_t = self.x_0/T_c*np.sinh(t/T_c) + self.vx_0*np.cosh(t/T_c)

        y_t = self.y_0*np.cosh(t/T_c) + T_c*self.vy_0*np.sinh(t/T_c)
        vy_t = self.y_0/T_c*np.sinh(t/T_c) + self.vy_0*np.cosh(t/T_c)

        return x_t, vx_t, y_t, vy_t

    def nextReferenceFootLocation(self, s_x, s_y, theta=0):
        if self.support_leg == 'left_leg': # then the next support leg is the right leg
            p_x_new = self.p_x + np.cos(theta)*s_x - np.sin(theta)*s_y
            p_y_new = self.p_y + np.sin(theta)*s_x + np.cos(theta)*s_y
        elif self.support_leg == 'right_leg': # then the next support leg is the left leg
            p_x_new = self.p_x + np.cos(theta)*s_x + np.sin(theta)*s_y
            p_y_new = self.p_y + np.sin(theta)*s_x - np.cos(theta)*s_y

        return p_x_new, p_y_new

    def nextState(self, s_x, s_y, theta=0):
        '''
        Calculate next final state at T_sup
        '''
        if self.support_leg == 'left_leg':
            bar_x_new = np.cos(theta)*s_x/2.0 - np.sin(theta)*s_y/2.0
            bar_y_new = np.sin(theta)*s_x/2.0 + np.cos(theta)*s_y/2.0
        elif self.support_leg == 'right_leg':
            bar_x_new = np.cos(theta)*s_x/2.0 + np.sin(theta)*s_y/2.0
            bar_y_new = np.sin(theta)*s_x/2.0 - np.cos(theta)*s_y/2.0
        return bar_x_new, bar_y_new

    def nextVel(self, bar_x=0, bar_y=0, theta=0):
        C = self.C
        S = self.S
        T_c = self.T_c

        bar_vx_new = np.cos(theta)*(1+C)/(T_c*S)*bar_x - np.sin(theta)*(C-1)/(T_c*S)*bar_y
        bar_vy_new = np.sin(theta)*(1+C)/(T_c*S)*bar_x + np.cos(theta)*(C-1)/(T_c*S)*bar_y

        return bar_vx_new, bar_vy_new

    def targetState(self, p_x, bar_x, bar_vx):
        x_d = p_x + bar_x
        vx_d = bar_vx

        return x_d, vx_d
    
    def modifiedFootLocation(self, a=1.0, b=1.0, x_d=0, vx_d=0, x_0=0, vx_0=0):
        C = self.C
        S = self.S
        T_c = self.T_c
        D = a*(C - 1)**2 + b*(S/T_c)**2

        p_x_star = -a*(C-1)*(x_d - C*x_0 - T_c*S*vx_0)/D - b*S*(vx_d - S*x_0/T_c - C*vx_0)/(T_c*D)

        return p_x_star

    def calculateFootLocationForNextStep(self, s_x=0.0, s_y=0.0, a=1.0, b=1.0, theta=0.0, x_0=0.0, vx_0=0.0, y_0=0.0, vy_0=0.0):
        self.s_x = s_x
        self.s_y = s_y

        # ----------------------------- calculate desired COM states and foot locations for the given s_x, s_y and theta
        # calculate desired foot locations
        # print(self.p_x, self.p_y)
        p_x_new, p_y_new = self.nextReferenceFootLocation(s_x, s_y, theta)
        # print('-- p_x_new=%.3f'%p_x_new, ', p_y_new=%.3f'%p_y_new)

        # calculate desired COM states
        bar_x, bar_y = self.nextState(s_x, s_y, theta)
        bar_vx, bar_vy = self.nextVel(bar_x, bar_y, theta)
        # print('-- bar_x=%.3f'%bar_x, ', bar_y=%.3f'%bar_y)
        # print('-- bar_vx=%.3f'%bar_vx, ', bar_vy=%.3f'%bar_vy)

        # calculate target COM state in the next step
        self.x_d, self.vx_d = self.targetState(p_x_new, bar_x, bar_vx)
        self.y_d, self.vy_d = self.targetState(p_y_new, bar_y, bar_vy)
        # print('-- x_d=%.3f'%self.x_d, ', vx_d=%.3f'%self.vx_d)
        # print('-- y_d=%.3f'%self.y_d, ', vy_d=%.3f'%self.vy_d)

        # ----------------------------- calculate modified foot locations based on the current actual COM states
        # correct the modified foot locations to minimize the errors
        self.p_x_star = self.modifiedFootLocation(a, b, self.x_d, self.vx_d, x_0, vx_0)
        self.p_y_star = self.modifiedFootLocation(a, b, self.y_d, self.vy_d, y_0, vy_0)
        # print('-- p_x_star=%.3f'%self.p_x_star, ', p_y_star=%.3f'%self.p_y_star)

    def switchSupportLeg(self):
        if self.support_leg == 'left_leg':
            print('\n---- switch the support leg to the right leg')
            self.support_leg = 'right_leg'
            COM_pos_x = self.x_t + self.left_foot_pos[0]
            COM_pos_y = self.y_t + self.left_foot_pos[1]
            self.x_0 = COM_pos_x - self.right_foot_pos[0]
            self.y_0 = COM_pos_y - self.right_foot_pos[1]
        elif self.support_leg == 'right_leg':
            print('\n---- switch the support leg to the left leg')
            self.support_leg = 'left_leg'
            COM_pos_x = self.x_t + self.right_foot_pos[0]
            COM_pos_y = self.y_t + self.right_foot_pos[1]
            self.x_0 = COM_pos_x - self.left_foot_pos[0]
            self.y_0 = COM_pos_y - self.left_foot_pos[1]

        self.t = 0
        self.vx_0 = self.vx_t
        self.vy_0 = self.vy_t

COM_pos = sim.getObjectPosition(COM, -1)
COM_vel = sim.getObjectVelocity(COM, -1)
left_foot_pos = sim.getObjectPosition(legr, -1)
right_foot_pos = sim.getObjectPosition(legl, -1)
x_left_foot, y_left_foot, z_left_foot = left_foot_pos
x_right_foot, y_right_foot, z_right_foot = right_foot_pos
print("Left Foot Position:", left_foot_pos)
print("Right Foot Position:", right_foot_pos)

x_left_foot = left_foot_pos[0]
y_left_foot = left_foot_pos[1]
z_left_foot = left_foot_pos[2]

x_right_foot = right_foot_pos[0]
y_right_foot = right_foot_pos[1]
z_right_foot = right_foot_pos[2]

lipm_model = LIPM3D()
lipm_model.initializeModel(COM_pos, x_left_foot, y_left_foot, z_left_foot, x_right_foot, y_right_foot, z_right_foot)

lipm_model.x_0 = COM_pos[0] - x_left_foot
lipm_model.vx_0 = COM_vel[0]
lipm_model.y_0 = COM_pos[1] - y_right_foot
lipm_model.vy_0 = COM_vel[1]

T_sup = 1.0
s_x = 80
s_y = 79
theta = 15

duration = 15
t = 0.0

while t < duration:
    lipm_model.step()
    lipm_model.calculateFootLocationForNextStep(s_x, s_y, theta)
    desired_positions = []
    desired_velocities = []

    for i in range(len(joint_handles)):
        desired_position = 0.0 + 0.2 * np.sin(2 * np.pi * 0.5 * t)
        desired_velocity = 0.2 * np.cos(2 * np.pi * 0.5 * t) * 2 * np.pi * 0.5

        desired_positions.append(desired_position)
        desired_velocities.append(desired_velocity)

    for i in range(len(joint_handles)):
        if i < len(joint_handles):
            desired_position = desired_positions[i]
            desired_velocity = desired_velocities[i]
        
            sim.setJointPosition(joint_handles[i], desired_position)
            sim.setJointTargetVelocity(joint_handles[i], desired_velocity)
    
    
    sim.setJointTargetPosition(legr, lipm_model.p_x_star)
    sim.setJointTargetPosition(legl, lipm_model.p_y_star)
    sim.setJointTargetVelocity(legr, lipm_model.vx_d)
    sim.setJointTargetVelocity(legl, lipm_model.vy_d)
    t += T_sup

sim.stopSimulation()
