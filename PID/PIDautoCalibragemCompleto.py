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
rotation_filed = robot_node.getField("rotation")

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
kp = 0.0  #10.0
ki = 0.0 #0.0001
kd = 0.0 #-50.0
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

# Variáveis para o aprendizado automático
list_pontos = []
geracao = 0
contadorGeracao = 0
media = 100
ultima_medida_pontos = [0.0, 0]
list_media = []
list_kp = []
list_geracoes = []
momento = []
indiceKp = 0
indiceKi = 0
indiceKd = 0
validadorP = True
validadorI = True
validadorD = True
contadorGrafico = 0


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

# Imprindo um gráfico do PID
def animateGeracoes():
    x = list_geracoes[0:99]
    y = list_media[0:99]
    plt.cla()
    plt.plot(x, y, label='Gerações')
    plt.xlabel('Geração')
    plt.ylabel('Melhor indivíduo')
    plt.title('Gerações Graph')
    plt.show()

def animateGeracoesKd():
    x = list_geracoes[100:199]
    y = list_media[100:199]
    plt.cla()
    plt.plot(x, y, label='Gerações')
    plt.xlabel('Geração')
    plt.ylabel('Melhor indivíduo')
    plt.title('Gerações Graph')
    plt.show()

def animateGeracoesKi():
    x = list_geracoes[200:299]
    y = list_media[200:299]
    plt.cla()
    plt.plot(x, y, label='Gerações')
    plt.xlabel('Geração')
    plt.ylabel('Melhor indivíduo')
    plt.title('Gerações Graph')
    plt.show()

def buscarMenor(lst):
    i = float("inf")
    for nr in lst:
        if nr < i:
            i = nr
    return i


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
        valorTranslation = [-2.4575, 0.0972027, -4.60195]
        valorRotation = [0.0245598, 0.999531, -0.0183043, -1.31491]

        # Modificando a posição do robô
        Field.setSFVec3f(trans_field, valorTranslation)
        # Gerando o erro de rotação
        Field.setSFRotation(rotation_filed, valorRotation)

    # Caso o valor zere, retornar para 360
    if bearing < 0.0:
        bearing = bearing + 360

    # PID CONTROLLER
    diferenca = (bearing) - ideal_value
    proporcional = diferenca * kp
    integral += diferenca * ki
    derivativo = (bearing - ultimaMedida) * kd
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
        leftWheel.setVelocity(4)  # 4
        rightWheel.setVelocity(4)  # 4

    values = trans_field.getSFVec3f()

    '''
    print("Velocidade Total")
    print(f"Velocidade D: {rightSpeed}")
    print(f"Velocidade E: {leftSpeed}")
    print()
    print(f"Erro gerado: {diferenca}")
    print(f"PID: {PID}")
    print()
    print("MY_ROBOT is at position: %g %g %g" % (values[0], values[1], values[2]))
    print("----------------")
    '''

    '''
    # Contador para imprimir o gráfico
    if contador > teste + 1000:
        teste = contador
        animate()
    contador += 1
    '''

    '''

    Posição inicial do robô
    x: -2.4575
    y: 0.0972027
    z: -4.60195

    '''

    # Contador para resetar a posição do robô
    if contadorReset > reset + 1000:
        # Fazendo a média dos erros
        somatoria = 0
        for x in range(len(list_error)):
            somatoria = abs(somatoria + list_error[x])
            # print(f"X: {list_error[x]}")
        print(f"Somatória: {somatoria}")
        media = somatoria / len(list_error)
        print(f"Média dos erros: {media}")
        list_media.append(media)
        list_mediaKd = list_media[100:]
        print(f"Geracao: {geracao}, Valores: {momento}")
        list_pontos.append(momento)
        if (geracao >= 0 and geracao < 100):
            if validadorP:
                momento = []
                validadorP = False
            melhor_individuo = buscarMenor(list_media)
            indiceKp = list_media.index(melhor_individuo)
            print(f"Melhor Individuo: {list_pontos[indiceKp]}, Indice: {indiceKp}")
            #Supervisor.simulationReset(robot)
            kp += 0.1  # 0.00001 1.273925
            momento = [media, kp]
        if (geracao >= 100 and geracao < 200):
            if validadorD:
                momento = []
                validadorD = False
            kp = list_pontos[indiceKp][1]
            melhor_individuo = buscarMenor(list_media[100:])
            indiceKd = list_media.index(melhor_individuo)
            print(f"Melhor Individuo: {list_pontos[indiceKd]}, Indice: {indiceKd}")
            #Supervisor.simulationReset(robot)
            kd += 0.1
            momento = [media, kd]
        if (geracao >= 200 and geracao < 300):
            if validadorI:
                momento = []
                validadorI
            kd = list_pontos[indiceKd][1]
            melhor_individuo = buscarMenor(list_media[200:])
            indiceKi = list_media.index(melhor_individuo)
            print(f"Melhor Individuo: {list_pontos[indiceKi]}, Indice: {indiceKi}")
            #Supervisor.simulationReset(robot)
            ki += 0.00001
            momento = [media, ki]
        if (geracao >= 300):
            print("PROCESSO FINALIZADO")
            ki = list_pontos[indiceKi][1]
            melhor_individuo = buscarMenor(list_media)
            indiceKi = list_media.index(melhor_individuo)
            print(f"Melhor Individuo: ")
            print(f"Kp: {kp}")
            print(f"Kd: {kd}")
            print(f"Ki: {ki}")
            animateGeracoes()
            animateGeracoesKd()
            animateGeracoesKi()
            Supervisor.simulationReset(robot)
            quit()
        list_geracoes.append(geracao)
        list_geracoesKd = list_geracoes[100:]
        geracao += 1
        reset = contadorReset
        valorTranslation = [-2.4575, 0.0972027, -4.60195]
        valorRotation = [0.0245598, 0.999531, -0.0183043, -1.31491]

        # Modificando a posição do robô
        Field.setSFVec3f(trans_field, valorTranslation)
        # Gerando erro de rotação
        Field.setSFRotation(rotation_filed, valorRotation)


        #print(f"Lista de pontos: {list_pontos}")
        ponto_atual = list_pontos[-1]
        ultima_medida_pontos = ponto_atual
        #print(f"Ponto atual: {ponto_atual}")
        #print(f"Ponto atual[0]: {ponto_atual[0]}")
        #print(f"ultima_medida_pontos[0]: {ultima_medida_pontos[0]}")
        print()

        '''
        if(geracao < 100):
            animateGeracoes()
        if geracao < 200 and geracao >= 100:
            animateGeracoesKd()
        if geracao <= 300 and geracao >=200:
            animateGeracoesKi()
        '''
        #Supervisor.simulationReset(robot)
        #quit()

    contadorReset += 1


