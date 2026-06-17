import math

class RigidBody6DOF:
    def __init__(self, start_lat, start_lon, start_alt):
        self.EARTH_RADIUS = 6378137.0
        self.GRAVITY = 9.81

        # Estado da Aeronave [u, v, w, p, q, r, lat, lon, alt, phi, theta, psi]
        self.state = [
            50.0, 0.0, 0.0,            # u, v, w (m/s) -> u=50m/s (~100 nós) inicial
            0.0, 0.0, 0.0,             # p, q, r (rad/s)
            start_lat, start_lon, start_alt, 
            0.0, 0.0, 0.0              # phi, theta, psi (rad)
        ]

        # ---------------------------------------------------------
        # PROPRIEDADES DA AERONAVE (Exemplo: Similar a um Cessna 172)
        # ---------------------------------------------------------
        self.mass = 1200.0             # Massa (kg)
        
        # Tensor de Inércia (kg*m^2) - Assumindo simetria (Ixy = Iyz = Ixz = 0)
        self.Ixx = 1285.0              # Inércia ao longo do eixo de rolagem
        self.Iyy = 1825.0              # Inércia ao longo do eixo de arfagem
        self.Izz = 2667.0              # Inércia ao longo do eixo de guinada

        # Geometria da Asa
        self.wing_area = 16.2          # Área da asa (m^2)
        self.wing_span = 11.0          # Envergadura (m)
        self.chord = 1.49              # Corda aerodinâmica média (m)

        # Coeficientes Aerodinâmicos Linearizados (Adimensionais)
        self.C_L0 = 0.28               # Sustentação com alfa zero
        self.C_L_alpha = 4.58          # Gradiente de sustentação por radiano de alfa
        self.C_D0 = 0.03               # Arrasto parasita
        self.e = 0.8                   # Fator de eficiência de Oswald
        self.AR = (self.wing_span**2) / self.wing_area # Aspect Ratio

        # Derivadas de Estabilidade e Controle
        self.C_m0 = -0.02              # Momento de arfagem base
        self.C_m_alpha = -0.89         # Estabilidade longitudinal
        self.C_m_q = -12.4             # Amortecimento de arfagem
        self.C_m_de = -1.28            # Eficiência do profundor

        self.C_l_p = -0.47             # Amortecimento de rolagem
        self.C_l_da = 0.17             # Eficiência do aileron
        
        self.C_n_beta = 0.071          # Estabilidade direcional (Weathercock)
        self.C_n_r = -0.15             # Amortecimento de guinada
        self.C_n_dr = -0.074           # Eficiência do leme

    def _get_air_density(self, alt_m):
        """Calcula a densidade do ar baseada na Atmosfera Padrão (ISA)."""
        T_sea = 288.15 # Kelvin
        P_sea = 101325.0 # Pascal
        L = 0.0065 # Taxa de decaimento de temperatura (K/m)
        R = 287.05 # Constante do gás para o ar

        T = T_sea - L * alt_m
        if T < 216.65: T = 216.65 # Troposfera limite
        
        P = P_sea * (T / T_sea)**5.25588
        rho = P / (R * T)
        return rho

    def _compute_derivatives(self, state, thrust_n, delta_a, delta_e, delta_r):
        """
        Derivadas RK4 com Forças Aerodinâmicas e Newton-Euler completas.
        Controles esperados em Radianos: delta_a (aileron), delta_e (elevator), delta_r (rudder)
        """
        u, v, w, p, q, r, lat, lon, alt, phi, theta, psi = state

        # 1. Velocidade do Ar e Ângulos Aerodinâmicos
        V_a = math.sqrt(u**2 + v**2 + w**2)
        if V_a < 0.1: V_a = 0.1 # Evita divisão por zero no solo
        
        alpha = math.atan2(w, u)            # Ângulo de Ataque (rad)
        beta = math.asin(v / V_a)           # Ângulo de Derrapagem (rad)

        # 2. Pressão Dinâmica (Q)
        rho = self._get_air_density(alt)
        Q = 0.5 * rho * (V_a**2)

        # 3. Forças Aerodinâmicas (Referencial do Vento)
        # Sustentação (Lift)
        C_L = self.C_L0 + (self.C_L_alpha * alpha) + (self.C_m_de * delta_e * 0.2)
        Lift = Q * self.wing_area * C_L

        # Arrasto (Drag) = Arrasto parasita + Arrasto induzido
        C_D_induced = (C_L**2) / (math.pi * self.e * self.AR)
        Drag = Q * self.wing_area * (self.C_D0 + C_D_induced)

        # Força Lateral (Sideforce)
        Y_force = Q * self.wing_area * (beta * -0.2)

        # 4. Transformação de Forças do Vento para o Corpo (Eixos X, Y, Z)
        # O Empuxo atua diretamente no eixo X do corpo.
        F_x_aero =  Lift * math.sin(alpha) - Drag * math.cos(alpha)
        F_z_aero = -Lift * math.cos(alpha) - Drag * math.sin(alpha)
        
        F_x = F_x_aero + thrust_n
        F_y = Y_force
        F_z = F_z_aero

        # 5. Momentos Aerodinâmicos (L, M, N)
        # Roll Moment (L)
        L_aero = Q * self.wing_area * self.wing_span * (
            (self.C_l_p * p * self.wing_span / (2 * V_a)) + 
            (self.C_l_da * delta_a)
        )

        # Pitch Moment (M)
        M_aero = Q * self.wing_area * self.chord * (
            self.C_m0 + 
            (self.C_m_alpha * alpha) + 
            (self.C_m_q * q * self.chord / (2 * V_a)) + 
            (self.C_m_de * delta_e)
        )

        # Yaw Moment (N)
        N_aero = Q * self.wing_area * self.wing_span * (
            (self.C_n_beta * beta) + 
            (self.C_n_r * r * self.wing_span / (2 * V_a)) + 
            (self.C_n_dr * delta_r)
        )

        # 6. Equações de Movimento Linear (Newton) - Incluindo Gravidade e Coriolis
        g_x = -self.GRAVITY * math.sin(theta)
        g_y =  self.GRAVITY * math.cos(theta) * math.sin(phi)
        g_z =  self.GRAVITY * math.cos(theta) * math.cos(phi)

        dot_u = (F_x / self.mass) + g_x - (q * w - r * v)
        dot_v = (F_y / self.mass) + g_y - (r * u - p * w)
        dot_w = (F_z / self.mass) + g_z - (p * v - q * u)

        # 7. Equações de Movimento Angular (Euler)
        dot_p = (L_aero + (self.Iyy - self.Izz) * q * r) / self.Ixx
        dot_q = (M_aero + (self.Izz - self.Ixx) * p * r) / self.Iyy
        dot_r = (N_aero + (self.Ixx - self.Iyy) * p * q) / self.Izz

        # 8. Cinemática Rotacional
        tan_theta = math.tan(theta) if abs(theta) < 1.5 else 0.0
        cos_theta = math.cos(theta) if abs(theta) < 1.5 else 0.001

        dot_phi = p + q * math.sin(phi) * tan_theta + r * math.cos(phi) * tan_theta
        dot_theta = q * math.cos(phi) - r * math.sin(phi)
        dot_psi = (q * math.sin(phi) + r * math.cos(phi)) / cos_theta

        # 9. Posição na Terra
        v_north = u * math.cos(theta) * math.cos(psi) - v * (math.cos(phi)*math.sin(psi) - math.sin(phi)*math.sin(theta)*math.cos(psi)) + w * (math.sin(phi)*math.sin(psi) + math.cos(phi)*math.sin(theta)*math.cos(psi))
        v_east = u * math.cos(theta) * math.sin(psi) + v * (math.cos(phi)*math.cos(psi) + math.sin(phi)*math.sin(theta)*math.sin(psi)) - w * (math.sin(phi)*math.cos(psi) - math.cos(phi)*math.sin(theta)*math.sin(psi))
        v_down = -u * math.sin(theta) + v * math.sin(phi)*math.cos(theta) + w * math.cos(phi)*math.cos(theta)

        cos_lat = math.cos(lat) if abs(math.cos(lat)) > 0.0001 else 0.0001
        
        dot_lat = v_north / self.EARTH_RADIUS
        dot_lon = v_east / (self.EARTH_RADIUS * cos_lat)
        dot_alt = -v_down 

        return [dot_u, dot_v, dot_w, dot_p, dot_q, dot_r, 
                dot_lat, dot_lon, dot_alt, dot_phi, dot_theta, dot_psi]

    def update(self, dt, thrust_n, delta_a, delta_e, delta_r):
        """
        Método de Integração RK4.
        delta_a: Aileron (+ direita desce)
        delta_e: Elevator (+ profundor desce = nariz cai)
        delta_r: Rudder (+ leme para a direita = guinada à direita)
        """
        y = self.state

        k1 = self._compute_derivatives(y, thrust_n, delta_a, delta_e, delta_r)
        y_k2 = [y[i] + 0.5 * dt * k1[i] for i in range(12)]
        k2 = self._compute_derivatives(y_k2, thrust_n, delta_a, delta_e, delta_r)
        y_k3 = [y[i] + 0.5 * dt * k2[i] for i in range(12)]
        k3 = self._compute_derivatives(y_k3, thrust_n, delta_a, delta_e, delta_r)
        y_k4 = [y[i] + dt * k3[i] for i in range(12)]
        k4 = self._compute_derivatives(y_k4, thrust_n, delta_a, delta_e, delta_r)

        for i in range(12):
            self.state[i] = y[i] + (dt / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i])

    # Propriedades de leitura (inalteradas)
    @property
    def lat(self): return self.state[6]
    @property
    def lon(self): return self.state[7]
    @property
    def alt(self): return self.state[8]
    @property
    def phi(self): return self.state[9]
    @property
    def theta(self): return self.state[10]
    @property
    def psi(self): return self.state[11]