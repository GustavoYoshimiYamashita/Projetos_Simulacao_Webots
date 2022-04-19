from controller import *
import math
import matplotlib.pyplot as plt

plt.style.use('ggplot')
from itertools import count

seconds = 0
teste = 0

robot = Robot()

# tempo que o código fica autalizando em milisegundos
TIME_STEP = 1

# velocidade máxima do robô
MAX_SPEED = 25.0

# valor máximo do sensor 0-1023 (10bits)
MAX_SENSOR_VALUE = 1024
MIN_DISTANCE = 1.0

# definindo os motores
leftWheel = robot.getDevice('left wheel')
rightWheel = robot.getDevice('right wheel')

# o motor pode girar de forma infinita
leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))

# variaveis do sensor sharp para a interplação exponencial
A = 118.67443195
t = 0.53449301
y0 = 8.76683547

# variaveis do PID
diferenca = 0.0
kp = 10.0
ki = 0.0001
kd = -10
proporcional = 0.0
integral = 0.0
derivativo = 0.0
PID = 0.0
ideal_value = 0
ultimaMedida = 0.0

# inicializando os sensores Sharp
ps = []
psNames = [
    'Sharps left', 'Sharps right', 'Sharps front'
]

# loop de leitura dos sharps
for i in range(3):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)


#  Função Map do arduino, regra de três
def _map(x, in_min, in_max, out_min, out_max):
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


list_seconds = []
list_error = []

index = count()

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


contador = 0
total = 0.0

medida1 = 0
medida2 = 0

import random
from datetime import datetime

now = datetime.now()
now2 = datetime.now()

# Gerando um erro no movimento do robô propositalmente para simular um ambiente dinâmico
def push_robot(push_force, current_direction):
    #error_velocity = lambda x: -(push_force - x)
    error_velocity = push_force
    directions = ['right', 'left']
    direction = random.choice(directions)
    vel = 0.0
    if direction == 'right' and direction != current_direction:
        #vel = error_velocity(rightWheel.getVelocity())
        vel = rightWheel.getVelocity() + error_velocity
        rightWheel.setVelocity(vel)
    elif direction == 'left' and direction != current_direction:
        #vel = error_velocity(leftWheel.getVelocity())
        vel = leftWheel.getVelocity() + error_velocity
        leftWheel.setVelocity(vel)
    else:
        print("Same direction")

    print(f"choice    = {direction}")
    print(f"error vel = {vel}")
    print(f"L vel     = {leftWheel.getVelocity()}")
    print(f"R vel     = {rightWheel.getVelocity()}")
    print("=" * 60)
    return direction

# Fazendo uma leitura de 50 valores do sensor
def get_sensor_data(sensor_object):
    data = []
    for _ in range(50):
        data.append(sensor_object.getValue())
    return data


leftWheel.setVelocity(5)
rightWheel.setVelocity(5)
direction = ""
APPLY_PID = True

# Iniciando a simulação
while robot.step(TIME_STEP) != -1:
    # print(leftSensor)
    psValues = []

    # Para os 3 sensores:
    for i in range(3):
        psValues.append(ps[i].getValue())
        sensor_data = get_sensor_data(ps[i])
        # distanciaSharp1 = y0 + A*exp(exp1)
        # exp1 = (-1*valorVoltsSharp1)/t
        # double A = 118.67443195
        # double t = 0.53449301
        # double y0 = 8.76683547
        # double exp1
        exp1 = (-1.0 * psValues[i]) / t
        psValues[i] = y0 + (A * math.exp(exp1))
    # print(psValues[0])
    # print(psValues[1])

    # PID CONTROLLER
    diferenca = (psValues[0] - psValues[1]) - ideal_value
    proporcional = diferenca * kp
    integral += diferenca * ki

    ultimaMedida = (medida1 - medida2)

    medida1 = psValues[0]
    medida2 = psValues[1]

    derivativo = (ultimaMedida - (psValues[0] - psValues[1])) * kd

    PID = proporcional + integral + derivativo
    # print(f"diferenca: {diferenca}")

    # Lógica do PID para a correção
    if PID < -0.5:
        if PID < -54:
            PID = -54
        PID = _map(PID, 0, -54, 0, 0.7)

        leftSpeed = (0.3 + PID) * MAX_SPEED
        rightSpeed = 0.3 * MAX_SPEED
    elif PID > 0.5:
        if PID > 54:
            PID = 54
        PID = _map(PID, 0, 54, 0, 0.7)
        leftSpeed = 0.3 * MAX_SPEED
        rightSpeed = (0.3 + PID) * MAX_SPEED
    else:
        integral = 0
        leftSpeed = 0.3 * MAX_SPEED
        rightSpeed = 0.3 * MAX_SPEED

    # print(f"left: {leftSpeed}")
    # print(f"right: {rightSpeed}")

    # setando a velocidade nominal
    if APPLY_PID:
        print('PID')
        print(f'leftSpeed = {leftSpeed}')
        print(f'rightSpeed = {rightSpeed}')
        print('*'*60)
        leftWheel.setVelocity(leftSpeed)
        rightWheel.setVelocity(rightSpeed)

    if (datetime.now() - now).seconds >= 5:
        now = datetime.now()
        direction = push_robot(0, direction)
        print("PID BLOQUEADO")
        #APPLY_PID = False

    if (datetime.now() - now2).seconds >= 5.001:
        now2 = datetime.now()
        #APPLY_PID = True




    list_error.append(diferenca)
    list_seconds.append(contador)

    # print(f"List error: {list_error}")
    # print(f"List seconds: {list_seconds}")

    if contador > teste + 200:
        teste = contador
        animate()

    contador += 1
