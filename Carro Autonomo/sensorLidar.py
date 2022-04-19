from controller import *
import numpy as np
import math
import pygame
from pygame import gfxdraw

from datetime import datetime

r = 5

# Definindo o tempo de atualização da tela
TIME_STEP = 32

# create the robot instance
robot = Robot()

# Velocidade máxima do robô
MAX_SPEED =  15.0 #12.3

# Definindo os motores
leftWheel = robot.getDevice('left wheel')
rightWheel = robot.getDevice('right wheel')

# O motor pode girar de forma infinita
leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))

# Iniciando o sensor Lidar
lidar = robot.getDevice("lidar")
Lidar.enable(lidar, TIME_STEP)
Lidar.enablePointCloud(lidar)

# Adicioando o sensor compass (Bússola) ao robô
compass = robot.getDevice('compass')
compass.enable(TIME_STEP)
direction = 0
initial_value = True

# Definindo a velocidade inicial como zero
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)

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

angle = 30

surface = pygame.display.set_mode((horizontal, vertical))

''''''''''''''''''''''''''''''

graus = 0.0

now = datetime.now().second

validador_movimento = True
bearing_atual = 0

# 0 -> left
# 1 -> right

def somar_grau(atual, valor):
    for i in range(valor):
        if atual == 360:
            atual = 0
        atual += 1
    return atual

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

    for event in pygame.event.get():
        if event.type == 256:
            pygame.quit()
            exit()

    surface.fill(black)
    pygame.draw.circle(surface, white, (centroX, centroY), 5)
    number_points = Lidar.getNumberOfPoints(lidar)
    grafico = []
    momento = []
    valor = []
    list_valorX = []
    list_valorY = []
    list_norte = []
    list_leste =[]
    list_oeste = []
    graus = 0.0
    #print(math.sin(math.radians(30)))

    for x in range(number_points):

        #Coletando dado polar
        imagem = lidar.getRangeImage()
        #print(imagem[x])
        valor.append(imagem[x])
        momento.append(x)
        grafico = [valor, momento]
        graus = graus + 2.8125
        rad = (graus * math.pi) / 180

        # Transformando em coordenada cartesiana
        # cosseno -> x
        # seno -> y
        retaX = math.cos(rad) * imagem[x]
        retaX = (retaX * 60) + centroX
        list_valorX.append(retaX)
        retaY = math.sin(rad) * imagem[x]
        retaY = (retaY * 60) + centroY
        list_valorY.append(retaY)
        #print(f"RetaX: {retaY}, RetaY: {retaY}")

        # line(surface, color, start_pos, end_pos)

        pygame.draw.circle(surface, white, (list_valorX[x], list_valorY[x]), 5)
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

        '''
        print(f"N: {len(list_norte)}")
        print(f"L: {len(list_leste)}")
        print(f"O: {len(list_leste)}")
        '''

        media_frente = 0
        media_esquerda = 0
        media_direita = 0
        parede_frente = False
        parede_direita = False
        parede_esquerda = False

        for i in range(len(list_norte)):
            media_frente = media_frente + list_norte[i]
            if list_norte[i] < 0.5:
                parede_frente = True

        for i in range(len(list_leste)):
            media_direita = media_direita + list_leste[i]
            if list_leste[i] < 0.6:
                parede_direita = True

        for i in range(len(list_oeste)):
            media_esquerda = media_esquerda + list_oeste[i]
            if list_oeste[i] < 0.6:
                parede_esquerda = True

        '''
        if parede_direita and validador_movimento:
            #print("Parede direita")
            leftWheel.setVelocity(-3)
            rightWheel.setVelocity(3)
        elif parede_esquerda and validador_movimento:
            #print("Parede esquerda")
            leftWheel.setVelocity(3)
            rightWheel.setVelocity(-3)
        elif parede_frente and validador_movimento:
            leftWheel.setVelocity(3)
            rightWheel.setVelocity(-3)
        else:
            leftWheel.setVelocity(3)
            rightWheel.setVelocity(3)
        '''





    pygame.display.update()
    pygame.display.flip()
    #quit()