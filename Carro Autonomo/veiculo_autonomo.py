from controller import *
import numpy as np
import math
import pygame
from pygame import gfxdraw
from datetime import datetime
import cv2
import matplotlib.pyplot as plt


# Definindo o tempo de atualização da tela
TIME_STEP = 10

# create the robot instance
robot = Robot()

# Velocidade máxima do robô
MAX_SPEED =  15.0 #12.3

# Devices
left_front_wheel = robot.getDevice('left_front_wheel')
right_front_wheel = robot.getDevice('right_front_wheel')
left_steer = robot.getDevice('left_steer')
right_steer = robot.getDevice('right_steer')
left_rear_wheel = robot.getDevice('left_rear_wheel')
right_rear_wheel = robot.getDevice('right_rear_wheel')

left_front_wheel.setPosition(float('inf'))
right_front_wheel.setPosition(float('inf'))
left_rear_wheel.setPosition(float('inf'))
right_rear_wheel.setPosition(float('inf'))

# Camera
camera = robot.getDevice('camera')

camera.enable(TIME_STEP)

# Lights
left_flasher = robot.getDevice('left_flasher')
right_flasher = robot.getDevice('right_flasher')
tail_lights = robot.getDevice('tail_lights')
work_head_lights = robot.getDevice('work_head_lights')
road_head_lights = robot.getDevice('road_head_lights')

# Iniciando o sensor Lidar
lidar = robot.getDevice("lidar")
Lidar.enable(lidar, TIME_STEP)
Lidar.enablePointCloud(lidar)
# Variáveis para o lidar
grafico = [0.0, 0]
momento = []
leitura = []
valor = []
list_valorX = []
list_valorY = []
tamanho_lists = []
parede_frente = False
parede_direita = False
parede_esquerda = False
media_direita = 0
media_esquerda = 0
media_frente = 0
# Lista para a região norte do robô
list_norte = []
# Lista para a região sul do robô
list_sul = []
# Lista para a região leste do robô
list_leste = []
# Lista para a região oeste do robô
list_oeste = []
graus = 0.0

''' Variáveis para o Pygame'''

# Inicializando cor
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
black = (0, 0, 0)
gray = (200, 200, 200)
white = (255, 255, 255)
vertical = 600
horizontal = 600
centroY = int(vertical/2)
centroX = int(horizontal/2)
rad = 0

angle = 30

surface = pygame.display.set_mode((horizontal, vertical))

''''''''''''''''''''''''''''''

# Variáveis do PID
diferenca = 0.0
#kp = 10.0
#ki = 0.0001
#kd = -50.0
kp = 7.19999999999999 #10.0
ki =  0.00000500000000#0.0001
kd = 7.19999999999999 #-50.0
proporcional = 0.0
integral = 0.0
derivativo = 0.0
PID = 0.0
ideal_value = 180
ultimaMedida = 0.0

# Adicioando o sensor compass (Bússola) ao robô
compass = robot.getDevice('compass')
compass.enable(TIME_STEP)
direction = 0
initial_value = True

def set_steering_angle(wheel_angle):
    steering_angle = wheel_angle
    left_steer.setPosition(steering_angle)
    right_steer.setPosition(steering_angle)

def set_speed(velocity):
    left_front_wheel.setVelocity(velocity)
    right_front_wheel.setVelocity(velocity)
    left_rear_wheel.setVelocity(velocity)
    right_rear_wheel.setVelocity(velocity)

def transformando_cartesiana(imagem, centroX, centroY, rad):
    retaX = math.cos(rad) * imagem[x]
    retaX = (retaX * 30) + centroX
    list_valorX.append(retaX)
    retaY = math.sin(rad) * imagem[x]
    retaY = (retaY * 30) + centroY
    list_valorY.append(retaY)

def leitura_camera(camera):
    camera.getImage()
    camera.saveImage("camera1.jpg", 100)
    img = cv2.imread("../camera1.jpg", cv2.IMREAD_COLOR)
    width = int(img.shape[1] * 2)
    height = int(img.shape[0] * 2)

    dim = (width, height)

    img1 = cv2.resize(img, dim, interpolation= cv2.INTER_AREA)

    return img1

# Prominent Arduino map function :)
def _map(x, in_min, in_max, out_min, out_max):
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def draw_map(valor):
    if not valor[x] > 8:

        #print(f"Valor: {valor[x]}, momento: {momento[x]}, tipo: {valor[x] > 8}")
        if graus >= 355 or graus <= 5:
            list_norte.append(valor[x])
            pygame.draw.line(surface, red, (centroX, centroY), (list_valorX[x], list_valorY[x]))
            pygame.draw.circle(surface, red, (list_valorX[x], list_valorY[x]), 5)
        if graus >= 10 and graus <= 30:
            list_leste.append(valor[x])
            pygame.draw.line(surface, green, (centroX, centroY), (list_valorX[x], list_valorY[x]))
            pygame.draw.circle(surface, green, (list_valorX[x], list_valorY[x]), 5)
        if graus >= 330 and graus <= 350:
            list_oeste.append(valor[x])
            pygame.draw.line(surface, blue, (centroX, centroY), (list_valorX[x], list_valorY[x]))
            pygame.draw.circle(surface, blue, (list_valorX[x], list_valorY[x]), 5)
    pygame.draw.circle(surface, white, (list_valorX[x], list_valorY[x]), 5)

def PID_correcao_angulo(PID):
    if (PID < -2):
        if (PID < -1500): PID = -1500
        PID = _map(PID, 0, -100, 0, 30)
        PID = float(PID / 100)
        set_steering_angle(PID)

    elif (PID > 2):
        if (PID > 1500): PID = 1500
        PID = _map(PID, 0, 100, 0, -30)
        PID = float(PID/100)
        set_steering_angle(PID)
    else:
        integral = 0.0
        derivativo = 0.0
        set_steering_angle(0)



while robot.step(TIME_STEP) != -1:

    set_speed(1)

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

    print(bearing)

    # PID CONTROLLER
    diferenca = (bearing) - ideal_value
    proporcional = diferenca * kp
    integral += diferenca * ki
    derivativo = (bearing - ultimaMedida) * kd
    ultimaMedida = bearing
    PID = proporcional + integral + derivativo

    PID_correcao_angulo(PID)

    img = leitura_camera(camera)

    cv2.imshow("camera1", img)



    for event in pygame.event.get():
        if event.type == 256:
            pygame.quit()
            cv2.destroyAllWindows()
            exit()

    surface.fill(black)
    pygame.draw.circle(surface, white, (centroX, centroY), 5)
    number_points = Lidar.getNumberOfPoints(lidar)
    #print(number_points)
    grafico = []
    momento = []
    valor = []
    list_valorX = []
    list_valorY = []
    list_norte = []
    list_leste = []
    list_oeste = []
    graus = 0.0
    # print(math.sin(math.radians(30)))

    for x in range(number_points):
        # Coletando dado polar
        imagem = lidar.getRangeImage()
        # print(imagem[x])
        valor.append(imagem[x])
        momento.append(x)
        grafico = [valor, momento]
        graus = graus + 2.8125
        rad = (graus * math.pi) / 180

        transformando_cartesiana(imagem, centroX, centroY, rad)

        #drawing map
        draw_map(valor)

    pygame.display.update()
    pygame.display.flip()

