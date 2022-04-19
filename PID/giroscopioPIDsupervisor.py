from controller import *
from controller import Motor
from datetime import datetime
import math
import matplotlib.pyplot as plt
from controller import Supervisor
import sys
from controller import Field

# Definindo o tempo de atualização da tela
TIME_STEP = 10

# create the robot instance
robot = Supervisor()

# do this once only
robot_node = robot.getFromDef("PIONEER_3DX")
if robot_node is None:
    sys.stderr.write("No DEF Pioneer 3-DX node found in the current world file\n")
    sys.exit(1)
trans_field = robot_node.getField("translation")

robot_node = robot.getFromDef("PIONEER_3DX")

# Velocidade máxima do robô
MAX_SPEED = 12.3  # 12.3

# Definindo os motores
leftWheel = robot.getDevice('left wheel')
rightWheel = robot.getDevice('right wheel')

# O motor pode girar de forma infinita
leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))

# Variáveis do PID
diferenca = 0.0
kp = 10.0
ki = 0.0001
kd = -50.0
proporcional = 0.0
integral = 0.0
derivativo = 0
PID = 0.0
ideal_value = 180
ultimaMedida = 0.0


# Prominent Arduino map function :)
def _map(x, in_min, in_max, out_min, out_max):
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


# Adicioando o sensor compass (Bússola) ao robô
compass = robot.getDevice('compass')
compass.enable(TIME_STEP)
direction = 0
initial_value = True

# variáveis para plotar no gráfico
list_seconds = []
list_error = []
contador = 0
teste = 0
contadorReset = 0
reset = 0


# Imprindo um gráfico do PID
def animate():
    x = list_seconds
    y = list_error
    plt.cla()
    plt.plot(x, y, label='PID')
    plt.xlabel('Time')
    plt.ylabel('error (cm)')
    plt.title('PID Graph')
    plt.show()


# Começando a velocidade do robô como zero para evitar movimento inicial indesejado
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)

rightSpeed = 0
leftSpeed = 0

# Iniciando a simulação
while robot.step(TIME_STEP) != -1:
    # Recebendo os valores da bússola
    valuesCompass = compass.getValues()
    # Transformando os valores em ângulos
    rad = math.atan2(valuesCompass[0], valuesCompass[2])
    bearing = (rad - 1.5708) / math.pi * 180

    # Iniciando o valor inicial do ângulo como o primeiro ângulo calculado
    if initial_value:
        initial_angle = bearing
        initial_value = False
    # Caso o valor zere, retornar para 360
    if bearing < 0.0:
        bearing = bearing + 360

    # PID CONTROLLER
    diferenca = (bearing) - ideal_value
    proporcional = diferenca * kp
    integral += diferenca * ki
    derivativo = (ultimaMedida - bearing) * kd
    ultimaMedida = bearing
    PID = proporcional + integral + derivativo

    list_error.append(diferenca)
    list_seconds.append(contador)

    '''
    Ainda é possível testar o controle adicionando uma segunda variável que suba de maneira proporcional com o PID e aplicar
    no motor que não está sendo aplicado o PID em si.

    Exemplo: 
    PID = _map(PID, 0, -1500, 4, 15)
    motor2 = _map(PID, 4, 15, 0, 7)
    leftWheel.setVelocity(PID)
    rightWheel.setVelocity(motor2)
    '''
    if (PID < -2):
        if (PID < -1500): PID = -1500
        PID = _map(PID, 0, -1500, 4, 12.3)
        motor2 = _map(PID, 4, 12.3, 0, 6)
        leftSpeed = PID
        leftWheel.setVelocity(PID)
        rightWheel.setVelocity(0)

    elif (PID > 2):
        if (PID > 1500): PID = 1500
        PID = _map(PID, 0, 1500, 4, 12.3)
        motor2 = _map(PID, 4, 12.3, 0, 6)
        rightSpeed = PID
        leftWheel.setVelocity(0)
        rightWheel.setVelocity(PID)
    else:
        integral = 0
        derivativo = 0
        leftWheel.setVelocity(4)  # 4
        rightWheel.setVelocity(4)  # 4

    values = trans_field.getSFVec3f()

    print("Velocidade Total")
    print(f"Velocidade D: {rightSpeed}")
    print(f"Velocidade E: {leftSpeed}")
    print()
    print(f"Erro gerado: {diferenca}")
    print(f"PID: {PID}")
    print()
    print("MY_ROBOT is at position: %g %g %g" % (values[0], values[1], values[2]))
    print("----------------")

    # Contador para imprimir o gráfico
    if contador > teste + 200:
        teste = contador
        animate()

    # Contador para resetar a posição do robô
    if contadorReset > reset + 1000:
        reset = contadorReset
        valor = [-2.4575, 0.0972027, -4.60195]
        # Modificando a posição do robô
        Field.setSFVec3f(trans_field, valor)

    contador += 1
    contadorReset += 1

    '''

    Posição inicial do robô
    x: -2.4575
    y: 0.0972027
    z: -4.60195

    '''


