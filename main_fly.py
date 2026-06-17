import socket
import time
import math
from fgnetfdm import FGNetFDM
from flySim_engine import RigidBody6DOF
import pygame # Importar pygame para o loop de eventos
from cessna172p_config import CESSNA_172P
from joystick_controller import JoystickController
from instrumentsPanel import VirtualPanel # <-- IMPORTAÇÃO DO PAINEL

# Altere invert_y para True se o nariz do avião subir quando empurrar o manche para a frente
joystick_ctrl = JoystickController(invert_y=True, invert_z=True, invert_throttle=True) # Inverte o eixo de yaw para que guinar à direita seja positivo

# Configurações de Rede e Simulação
UDP_IP = "127.0.0.1"
UDP_PORT = 5500
HZ = 60
DT = 1.0 / HZ

# Instanciar a Rede
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Posição inicial (Ex: Sorocaba)
lat_inicial = math.radians(-23.4709)
lon_inicial = math.radians(-47.4851)
alt_inicial = 1500.0 # Metros

# Instanciar a Estrutura Binária do FG e o Motor Físico
fdm = FGNetFDM()
sim = RigidBody6DOF(lat_inicial, lon_inicial, alt_inicial)

# Inicializar a janela do Painel Virtual
painel = VirtualPanel() # <-- INICIALIZAÇÃO DO PAINEL

# Sincronizar propriedades da aeronave com as constantes do Cessna 172P definidas no config
sim.mass = CESSNA_172P["mass"]
sim.wing_area = CESSNA_172P["wing_area"]
sim.wing_span = CESSNA_172P["wing_span"]
sim.chord = CESSNA_172P["chord"]
sim.AR = (sim.wing_span**2) / sim.wing_area
sim.Ixx = CESSNA_172P["ixx"]
sim.Iyy = CESSNA_172P["iyy"]
sim.Izz = CESSNA_172P["izz"]
sim.C_D0 = CESSNA_172P["cd0"]
sim.C_L_alpha = CESSNA_172P["cl_alpha"]
sim.C_L0 = CESSNA_172P["cl0"]
sim.e = CESSNA_172P["oswald_eff"]

print(f"A iniciar simulação 6DOF a {HZ}Hz...")

try:
    while True:
        start_time = time.time()

        # ---------------------------------------------------------
        # COMANDOS DA SIMULAÇÃO
        
        for event in pygame.event.get():
            joystick_ctrl.handle_event(event)

        empuxo = 0.0  # Newtons
        cmd_roll = 0.0  # Comando positivo = rolar para a direita
        cmd_pitch = 0.0 # Comando positivo = nariz para cima
        cmd_yaw = 0.0   # Comando positivo = guinar para a direita        
        
        # COMANDOS DA SIMULAÇÃO (Hardware-in-the-loop via Joystick ou fallback)
        if joystick_ctrl.is_connected():
            throttle_input, roll_input, pitch_input, yaw_input = joystick_ctrl.get_controls()
            empuxo = throttle_input * CESSNA_172P["max_thrust"] # Mapeia 0-1 para 0-max_thrust
            
            # Comandos em radianos para a física (baseados nos limites reais da aeronave)
            cmd_roll = roll_input * math.radians(CESSNA_172P["limits"]["aileron"])
            cmd_pitch = pitch_input * math.radians(CESSNA_172P["limits"]["elevator"])
            cmd_yaw = yaw_input * math.radians(CESSNA_172P["limits"]["rudder"])
            
            # Valores normalizados (-1 a 1) para a visualização no FlightGear
            surf_aileron, surf_elevator, surf_rudder = roll_input, pitch_input, yaw_input

        else:
            # Fallback para valores padrão ou para um voo "autônomo" simples
            empuxo = CESSNA_172P["max_thrust"] * 0.7 # 70% de empuxo para manter sustentação
            cmd_roll = 0.0 # Sem roll
            cmd_pitch = 0.0 # Sem pitch
            cmd_yaw = 0.0 # Sem yaw
            surf_aileron = surf_elevator = surf_rudder = 0.0
        # ---------------------------------------------------------

        # Avança a física em 1 timestep (dt)
        sim.update(DT, empuxo, cmd_roll, cmd_pitch, cmd_yaw)

        # Mapeamento do Estado Físico para a Estrutura Binária do FlightGear
        fdm.latitude = sim.lat
        fdm.longitude = sim.lon
        fdm.altitude = sim.alt
        fdm.phi = sim.phi
        fdm.theta = sim.theta
        fdm.psi = sim.psi
        phi, theta, psi = sim.phi, sim.theta, sim.psi

        # Extrair velocidades e taxas para o FlightGear (conversão de unidades)
        u, v, w = sim.state[0], sim.state[1], sim.state[2]
        p, q, r = sim.state[3], sim.state[4], sim.state[5]
        v_air = math.sqrt(u**2 + v**2 + w**2)
        
        # --- CÁLCULOS PARA O PAINEL VIRTUAL ---
        # True Airspeed (TAS) - Velocidade real em relação à massa de ar
        tas_knots = v_air * 1.94384 
        
        # Indicated Airspeed (IAS) - Velocidade medida pelo pitot (afetada pela densidade do ar)
        alt_pes = sim.alt * 3.28084
        densidade_ratio = max(0.0, 1.0 - (0.000006875 * alt_pes)) ** 4.256
        ias_knots = tas_knots * math.sqrt(densidade_ratio)

        # Atualiza a interface gráfica do painel (Pitch, Roll, IAS, TAS)
        painel.update(theta, phi, ias_knots, tas_knots)
        # --------------------------------------

        # O instrumento visual do FlightGear (vcas) baterá com o seu painel
        fdm.vcas = ias_knots
        
        fdm.phidot = p                  # rad/s
        fdm.thetadot = q                # rad/s
        fdm.psidot = r                  # rad/s
        fdm.v_body_u = u * 3.28084      # m/s para ft/s
        fdm.v_body_v = v * 3.28084
        fdm.v_body_w = w * 3.28084
        fdm.alpha = math.atan2(w, u) if u != 0 else 0.0
        fdm.beta = math.asin(v / v_air) if v_air > 0.1 else 0.0

        # Velocidades locais e Taxa de Subida (Climb Rate)
        v_down = -u * math.sin(theta) + v * math.sin(phi)*math.cos(theta) + w * math.cos(phi)*math.cos(theta)
        fdm.v_north = (u * math.cos(theta) * math.cos(psi)) * 3.28084 # Simplificado
        fdm.climb_rate = -v_down * 3.28084 # m/s para ft/s (usado pelo VSI)
        
        # O FlightGear usa as superfícies visualmente, podemos enviar os comandos
        # normalizados para que a animação 3D use o curso total
        fdm.left_aileron = -surf_aileron
        fdm.right_aileron = surf_aileron
        fdm.elevator = surf_elevator
        fdm.rudder = surf_rudder

        # Empacota em 408 bytes e dispara
        payload = fdm.pack()
        sock.sendto(payload, (UDP_IP, UDP_PORT))

        # Controlo de tempo rigoroso para manter os 60Hz determinísticos
        elapsed = time.time() - start_time
        sleep_time = DT - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print("\nSimulação 6DOF encerrada.")
finally:
    sock.close()
    pygame.quit() # Garante que a janela do painel fecha graciosamente