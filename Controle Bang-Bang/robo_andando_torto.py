import time

from controller import *
from controller import DistanceSensor
import math
import matplotlib.pyplot as plt
plt.style.use('ggplot')
from datetime import datetime
from matplotlib.animation import FuncAnimation
from itertools import count
import numpy as np
import random

from datetime import datetime

seconds = 0
teste = 0

robot = Robot()

#tempo que o código fica autalizando em milisegundos
TIME_STEP = 10
#velocidade máxima do robô
MAX_SPEED = 10

#valor máximo do sensor 0-1023 (10bits)
MAX_SENSOR_VALUE = 1024
MIN_DISTANCE = 1.0

#definindo os motores
leftWheel = robot.getDevice('left wheel')
rightWheel = robot.getDevice('right wheel')


#o motor pode girar de forma infinita
leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))

#variaveis sharp exponencial
A = 118.67443195
t = 0.53449301
y0 = 8.76683547

#variaveis do PID

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


#  Prominent Arduino map function :)
def _map(x, in_min, in_max, out_min, out_max):
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

list_seconds = []
list_error = []

index = count()
contador_graph = 0

def animate():
    x = list_seconds
    y = list_error
    plt.cla()
    plt.plot(x, y, label='S')
    plt.xlabel('Time')
    plt.ylabel('Erro graus')
    plt.title('Gráfico de erros')
    plt.show()

contador = 0
total = 0.0

medida1 = 0
medida2 = 0

now = datetime.now()
left_error = 0
right_error = 0
leftSpeed = 0
rightSpeed = 0
contador = 0
atual = 0

# Adicioando o sensor compass (Bússola) ao robô
compass = robot.getDevice('compass')
compass.enable(TIME_STEP)
direction = 0
initial_value = True

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

    # setando a velocidade nominal
    if(contador > atual + 1):
        atual = contador
        now = datetime.now()
        left_error = random.randrange(0, 50, 1)
        right_error = random.randrange(0, 50, 1)
        negative = random.randrange(0, 2)
        if negative:
            left_error = left_error * -1
        print(f"Negative: {negative}")
        left_error = float(left_error/100)
        right_error = float(right_error/100)

        leftSpeed = float((0.5 + left_error) * MAX_SPEED)
        print(f"leftSpeed: {leftSpeed}")
        rightSpeed = float(0.5 * MAX_SPEED)
        print(f"rightSpeed: {rightSpeed}")
    leftWheel.setVelocity(leftSpeed)
    rightWheel.setVelocity(rightSpeed)
    contador += 1

    list_error.append(diferenca)
    list_seconds.append(contador_graph)

    # Contador para imprimir o gráfico
    if contador_graph > teste + 200:
        teste = contador_graph
        animate()
    contador_graph += 1



