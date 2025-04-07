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
        self.alpha = 0       
        self.u = 100
        self.v = 0
        self.w = 0
        self.theta = 5
        self.phi = 0
        self.psi = 0
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


            #x = self.x + dt*(self.u*math.cos(psi)*math.cos(theta) - self.v*math.sin(psi) + self.w* math.cos(theta))
            self.x = self.x + dt * (self.u * math.cos(psi) * math.cos(theta) - self.v * math.sin(psi) + self.w * math.sin(theta))# + dt * self.x_advance
            self.y = self.y + dt * (self.u * math.sin(psi) * math.cos(theta) + self.v * math.cos(psi)) #+ dt * self.y_advance
            self.z = self.z + dt * (-self.u * math.sin(theta) + self.w *math.cos(theta)) # dt * self.z_advance
            
            
            alpha = math.atan2(self.w, self.u) # radianos
               
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
        ax.set_xlabel('X ')# '''+ str(theta)''')
        ax.set_ylabel('Y ')# '''+ str(psi)''')
        ax.set_zlabel('Z ')# ''' + str(phi)''')
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






