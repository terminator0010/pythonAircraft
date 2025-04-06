import matplotlib.pyplot as plt
import numpy as np
import math


maxtime = 2
dt = 0.1
simtime = []

d2r = np.deg2rad

class FlightSimulation():
    
    def __init__(self):
        self.flightData()
        self.plotSim()
        plt.show()
        exit()

    
    
    def flightData(self):
        self.x = 0
        self.y = 0
        self.z = 0 
        self.alpha = 0       
        self.u = 200
        self.v = 0
        self.w = 0
        self.theta = 20
        self.phi = 0
        self.psi = 0
        self.xlim, self.ylim, self.zlim = 500, 500, 500  # Ajuste conforme necessário
        self.x_advance = 3
        self.y_advance = 0
        self.z_advance = 0
        self.x_values = []
        self.y_values = []
        self.z_values = []
     

    def plotSim(self):
        # Criar a figura e o eixo 3D antes do loop
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d') 
            
        for t in range(int(maxtime/dt) +1):
            
            theta = d2r(self.theta* np.cos(t / 2))
            phi = d2r(self.phi* np.cos(t / 2))
            psi = d2r(self.psi* np.cos(t / 2))
                           
            #x = self.x + dt*(self.u*math.cos(psi)*math.cos(theta) - self.v*math.sin(psi) + self.w* math.cos(theta))
            self.x += dt * (self.u * math.cos(psi) * math.cos(theta) - self.v * math.sin(psi) + self.w * math.sin(theta)) + dt * self.x_advance
            self.y += dt * (self.y + dt *(self.v*math.sin(psi)*math.cos(theta) + self.v*math.cos(psi))) + dt * self.y_advance
            self.z += dt * (self.z + dt*(-1*self.u*math.sin(theta) + self.w *math.cos(theta))) + dt * self.z_advance
            
            
            self.alpha = math.atan2(self.w, self.u) # radianos
               
            self.x_values.append(self.x + self.alpha) #replace
            self.y_values.append(self.y) #replace
            self.z_values.append(self.z) #replace

            ax.cla()  # Limpa o eixo para nova atualização
            ax.plot(self.x_values, self.y_values, self.z_values, label='Trajetória')
            ax.set_xlim([-1 *self.xlim, self.xlim])
            ax.set_ylim([-1 *self.ylim, self.ylim])
            ax.set_zlim([0, self.zlim])  # Mantendo z positivo para simular altitude 
            ax.scatter(self.x_values[0], self.y_values[0], self.z_values[0], color='red', marker='o', label='Início')
            ax.scatter(self.x_values[-1], self.y_values[-1], self.z_values[-1], color='green', marker='o', label='Atual')
            ax.set_label(theta)

            # Configuração dos rótulos
            ax.set_title(f'Trajetória em tempo real')
            ax.set_xlabel('X ' + str(theta))
            ax.set_ylabel('Y ' + str(psi))
            ax.set_zlabel('Z ' + str(phi))
            ax.legend()

            plt.pause(0.01)  # Pequena pausa para atualizar a animação
            #print(simtime[t])
            print(theta)
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






