"""
Especificações Técnicas e Aerodinâmicas: Cessna 172P
Fontes: Manuais de Operação (POH) e dados do projeto JSBSim.
"""

CESSNA_172P = {
    # --- Propriedades Geométricas ---
    "wing_area": 16.2,            # S: Área da asa (m²)
    "wing_span": 11.0,            # b: Envergadura (m)
    "chord": 1.49,                # c: Corda média aerodinâmica (m)
    
    # --- Massas e Inércias ---
    "mass": 1043.0,               # m: Massa máxima de decolagem (kg)
    "empty_mass": 757.0,          # Massa vazia aproximada (kg)
    
    # Momentos de Inércia (kg*m²) - Estimados para o eixo do corpo
    "ixx": 1285.6,                # Inércia de Roll (rolagem)
    "iyy": 1824.9,                # Inércia de Pitch (arfagem)
    "izz": 2666.9,                # Inércia de Yaw (guinada)
    "ixz": 0.0,                   # Produto de inércia (geralmente desprezível em simetria)

    # --- Performance do Motor (Lycoming IO-360-L2A) ---
    "max_thrust": 2600.0,         # Empuxo estático máximo aproximado (Newtons)
    "max_hp": 160.0,              # Potência máxima (Horsepower)
    "rpm_max": 2700.0,            # RPM Máxima

    # --- Coeficientes Aerodinâmicos (Adimensionais) ---
    "cd0": 0.0310,                # Parasitic Drag (Arrasto parasita com flaps recolhidos)
    "cl_alpha": 4.58,             # Lift Slope (Gradiente de sustentação por radiano)
    "cl0": 0.31,                  # Coeficiente de sustentação com ângulo de ataque zero
    "oswald_eff": 0.8,            # Fator de eficiência de Oswald (e)
    
    # --- Limites Operacionais ---
    "v_ne": 158.0,                # Velocity Never Exceed (Nós) - 293 km/h
    "v_no": 127.0,                # Máxima velocidade de cruzeiro estrutural (Nós)
    "v_fe": 85.0,                 # Máxima velocidade com flaps estendidos (Nós)
    "v_so": 33.0,                 # Velocidade de estol com flaps (Nós)
    
    # --- Superfícies de Controle (Deflexões máximas em graus) ---
    "limits": {
        "elevator": 28.0,         # Subida (28°) / Descida (21°)
        "aileron": 20.0,          # Cima (20°) / Baixo (15°)
        "rudder": 24.0,           # Esquerda/Direita (24°)
        "flaps": 30.0             # Extensão total (30°)
    },

    # Unidades de conversão úteis para a simulação
    "units": "SI (Meters, Kilograms, Seconds)",
    "reference_datum": "Firewall / Centerline"
}