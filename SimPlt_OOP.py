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
        self.m = 1000 #Massa kg
        self.g = 9.81
        self.ib = 1
        self.jb = 1
        self.kb = 1
        self.theta = 0
        self.phi = 0
        self.psi = 0
        self.p = 0
        self.q = 0
        self.r = 0
        self.xlim, self.ylim, self.zlim = 1300, 1300, 300  # Ajuste conforme necessário   
        self.x_values = []
        self.y_values = []
        self.z_values = []
        
    
    def atmospheric(self):
        self.rho = 1.225
        self.P0_PA = 101325
        self.k = -0.0065
        self.Gamma = 1.4
        self.T0 = 288.15
        self.R = 287    
        
        
        self.Temp_k = -self.z*self.k + self.T0
        self.Pressura_PA = (self.Temp_k * (1/self.T0)+math.pow(-self.g/(-self.k*self.Gamma))*self.P0_PA)*(math.exp((-self.g/self.R)/self.Temp_k)*(self.z-11000))
        self.SpeedofSound_mps = math.sqrt(self.Temp_k*self.R*self.Gamma)
        self.Rho_kgpm3 = self.Pressura_PA/self.Temp_k * self.R
        
        

    def flightMechannics(self):
        # Criar a figura e o eixo 3D antes do loop           
        for t in range(int(maxtime/dt) +1):
            
            weight_N = self.m * self.g
            
            theta = d2r(-self.theta) 
            phi = d2r(self.phi)
            psi = d2r(self.psi)
            gamma = d2r(math.atan2(self.w, self.u))
            
            i2 = self.ib
            k1 = -math.cos(theta)*self.ib + math.cos(theta)*math.sin(phi)*self.jb + math.cos(phi) * math.cos(theta)*self.kb
            j2 = math.cos(phi)*self.jb - math.sin(phi)*self.kb

            self.p = (phi - self.psi*math.sin(theta))
            self.q = (theta*math.cos(phi )+ phi*math.cos(phi)*math.sin(phi))
            self.r = (psi*math.cos(phi)*math.cos(theta)- theta*math.sin(phi))

            phi = self.p + self.q*math.sin(phi)*math.tan(theta) + self.r*math.cos(phi)*math.tan(theta)
            theta = self.q*math.cos(phi) - self.r*math.sin(phi)
            psi = (self.q*math.sin(phi) + self.r*math.cos(phi))/math.cos(theta)
            '''
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
            '''

            #x = self.x + dt*(self.u*math.cos(psi)*math.cos(theta) - self.v*math.sin(psi) + self.w* math.cos(theta))
            self.x = (self.x + dt * (self.u * math.cos(psi) * math.cos(theta) + self.v * (math.sin(phi) * math.sin(psi) * math.cos(psi)) + self.w * (math.cos(phi) * math.sin(theta) * math.cos(psi) + math.sin(phi)*math.sin(psi))))
            self.y = (self.y + dt * (self.u * math.sin(psi) * math.cos(theta) + self.v * (math.cos(psi) * math.sin(theta) * math.sin(phi)) + self.w * (math.cos(phi) * math.sin(theta) * math.sin(psi) - math.sin(phi)* math.cos(psi))))
            self.z = (self.z + dt * (-self.u * math.sin(theta) + self.v * math.cos(theta)* math.sin(phi) * self.w * math.cos(theta)* math.cos(phi)))
            
            
               
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






