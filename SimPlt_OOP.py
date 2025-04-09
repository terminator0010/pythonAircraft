import matplotlib.pyplot as plt
import numpy as np
import math


maxtime = 10
dt = 0.01
simtime = []

d2r = np.deg2rad

class FlightSimulation():
    
    def __init__(self):
        self.flightData()
        self.flightMechannics()
        self.plotSim()
        exit()

    
    
    def flightData(self):
        self.x = 100
        self.y = 50
        self.z = 100    
        self.u = 100
        self.v = 0
        self.w = 0
        self.m = 0 #Massa kg
        self.ib = 0
        self.jb = 0
        self.kb = 0
        self.theta = 5
        self.phi = 0
        self.psi = 0
        self.p = 0
        self.q = 0
        self.r = 0
        self.xlim, self.ylim, self.zlim = 1300, 1300, 300  # Ajuste conforme necessário
        self.x_advance = 0
        self.y_advance = 0
        self.z_advance = 0
        self.x_values = []
        self.y_values = []
        self.z_values = []
        
     

    def flightMechannics(self):
        # Criar a figura e o eixo 3D antes do loop           
        for t in range(int(maxtime/dt) +1):
            
            theta = d2r(-self.theta) 
            phi = d2r(self.phi)
            psi = d2r(self.psi)
            gamma = d2r(math.atan2(self.w, self.u))

            i2 = self.ib
            k1 = -math.cos(theta)*self.ib + math.cos(theta)*math.sin(phi)*self.jb + math.cos(phi) * math.cos(theta)*self.kb
            j2 = math.cos(phi)*self.jb - math.sin(phi)*self.kb

            p = (phi_dot - psi_dot*math.sin(theta))
            q = (theta_dot*math.cos(phi )+ phi_dot*math.cos(phi)*math.sin(phi))
            r = (psi_dot*math.cos(phi)*math.cos(theta)- theta_dot*math.sin(phi))

            phi_dot = p + q*math.sin(phi)*math.tan(theta) + r*math.cos(phi)*math.tan(theta)
            theta_dot = q*math.cos(phi) - r*math.sin(phi)
            psi_dot = (q*math.sin(phi) + r*math.cos(phi))/math.cos(theta)

            u_dot = r*self.v - q*self.w+ TransMotion(1)/self.m
            v_dot = p*self.p - r*self.u + TransMotion(2)/self.m
            w_dot = q*self.u - p*self.v + TransMotion(3)/self.m

            X_dot = [u_dot, v_dot, w_dot]
            X_matrix = [self.u, self.v, self.w]
            #omega = self.ib*p + self.jb*q + self.kb*r
            
            
            omega = [p,q,r]

            TransMotion = self.m * (X_dot + np.cross(omega, X_matrix))

            alpha = theta - gamma
            V = math.sqrt(pow(self.u)+pow(self.v)+pow(self.w,))
            beta = math.asin(self.v,V)

            #x = self.x + dt*(self.u*math.cos(psi)*math.cos(theta) - self.v*math.sin(psi) + self.w* math.cos(theta))
            self.x = self.x + dt * (self.u * math.cos(psi) * math.cos(theta) + self.v * (math.sin(phi) * math.sin(psi) * math.cos(psi)) + self.w * (math.cos(phi) * math.sin(theta) * math.cos(psi) + math.sin(phi)*math.sin(psi)))# + dt * self.x_advance
            self.y = self.y + dt * (self.u * math.sin(psi) * math.cos(theta) + self.v * (math.cos(psi) * math.sin(theta) * math.sin(phi)) + self.w * (math.cos(phi) * math.sin(theta) * math.sin(psi) - math.sin(phi)* math.cos(psi))) #+ dt * self.y_advance
            self.z = self.z + dt * (-self.u * math.sin(theta) + self.v * math.cos(theta)* math.sin(phi) * self.w * math.cos(theta)* math.cos(phi)) # dt * self.z_advance
            
            
               
            self.x_values.append(self.x) #replace
            self.y_values.append(self.y) #replace
            self.z_values.append(self.z) #replace


    def plotSim(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d') 
        ax.cla()  # Limpa o eixo para nova atualização
        ax.plot(self.x_values, self.y_values, self.z_values, label='Trajetória')
        ax.set_xlim([-1 *self.xlim, self.xlim])
        ax.set_ylim([-1 *self.ylim, self.ylim])
        ax.set_zlim([0, self.zlim])  # Mantendo z positivo para simular altitude 
        ax.scatter(self.x_values[0], self.y_values[0], self.z_values[0], color='red', marker='>', label='Início')
        ax.scatter(self.x_values[-1], self.y_values[-1], self.z_values[-1], color='green', marker='>', label='Atual')
        ax.set_label(self.theta)

        # Configuração dos rótulos
        ax.set_title(f'Trajetória em tempo real')
        ax.set_xlabel('Y ' + 'Pitch')
        ax.set_ylabel('X ' + 'Roll')
        ax.set_zlabel('Z ' + 'Yaw')
        ax.legend()

        plt.pause(0.01)  # Pequena pausa para atualizar a animação
        plt.show()
        #print(simtime[t])
        #print(self.flightMechannics.theta)
        print(self.theta)
        #print(frac_values)
    
    


# Coordenadas inicial e final

'''
# Criar a figura 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(self.x_values, self.y_values, self.z_values, label='Trajetória')
ax.scatter([self.x_values[0]], [self.y_values[0]], [self.z_values[0], color='red', marker='o', label='Início')
ax.scatter([self.x_values[int(maxtime/dt)]], [self.y_values[int(maxtime/dt)]], [self.z_values[int(maxtime/dt)]], color='green', marker='x', label='Fim')

# Configuração dos rótulos
title = f'Trajetória de ({int(self.x_values[0])}, {int(self.y_values[0])}, {int(self.z_values[0])}) até ({(self.x_values[maxtime])}, {(self.y_values[maxtime])}, {(self.z_values[maxtime])})'
ax.set_title(title)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
'''

if __name__ == '__main__':
    sim = FlightSimulation()






