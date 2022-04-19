from controller import Robot
from controller import Motor
from datetime import datetime
import math
import matplotlib.pyplot as plt

plt.style.use('ggplot')

TIME_STEP = 10
MAX_SPEED = 6.28

# create the robot instance
robot = Robot()

#velocidade máxima do robô
MAX_SPEED = 15

#definindo os motores
leftWheel = robot.getDevice('left wheel')
rightWheel = robot.getDevice('right wheel')

#o motor pode girar de forma infinita
leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))

minVal = -2
maxVal = 9

x = 0.0
y = 0.0
z = 0.0

xAng = 0
yAng = 0
zAng = 0

#  Prominent Arduino map function :)
def _map(x, in_min, in_max, out_min, out_max):
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

list_seconds = []
list_error = []
contador = 0
teste = 0

# Imprindo um gráfico do PID
def animate():

    x = list_seconds
    y = list_error
    plt.cla()
    plt.plot(x, y, label='PID')
    plt.xlabel('Time')
    plt.ylabel('error graus')
    plt.title('Gráfico de erros')
    plt.show()


compass = robot.getDevice('compass')
compass.enable(TIME_STEP)
direction = 0
initial_value = True

leftSpeed = 0.5 * MAX_SPEED
rightSpeed = 0.5 * MAX_SPEED

ideal_value = 180

# Iniciando a simulação
while robot.step(TIME_STEP) != -1:
    valuesCompass = compass.getValues()
    rad = math.atan2(valuesCompass[0], valuesCompass[2])
    bearing = (rad - 1.5708) / math.pi * 180
    if initial_value:
        initial_angle = 180
        initial_value = False
    if bearing < 0.0:
        bearing = bearing + 360

    #Erro = diferenca
    diferenca = (bearing) - ideal_value

    if diferenca < 0:
        leftSpeed = 1.5 * MAX_SPEED
        rightSpeed = 0
        leftWheel.setVelocity(leftSpeed)
        rightWheel.setVelocity(rightSpeed)
    elif diferenca > 0:
        rightSpeed = 1.5 * MAX_SPEED
        leftSpeed = 0
        leftWheel.setVelocity(leftSpeed)
        rightWheel.setVelocity(rightSpeed)
    else:
        leftWheel.setVelocity(10)
        rightWheel.setVelocity(10)

    print("----------------")
    print("Velocidade Total")
    print(f"Velocidade D: {rightSpeed}")
    print(f"Velocidade E: {leftSpeed}")
    print("----------------")
    print()


    list_error.append(diferenca)
    list_seconds.append(contador)

    if contador > teste + 200:
        teste = contador
        animate()

    contador += 1


