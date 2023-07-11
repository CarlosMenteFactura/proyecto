import torch
import numpy as np
import cv2
import time
from datetime import datetime
import math
#import EscrituraServidor
from Analizador import AnalizadorDiametro
from Conexion import Conexion
from easymodbus.modbusClient import ModbusClient
#import matplotlib.pyplot as plt '''comentado por hector 19012023_lib no utilizada'''
from threading import Thread, Event
# import pywhatkit as wa #Necesita internet para iniciar
import pyrealsense2
from realsense_depth import *

class Vision:
    '''Función utilizada para reconectar camara cuando esta es cambiada o desconectada'''
    def conexionCamara(self):
        '''
            Descripcion
            -------
            Funcion utilizada para reconectar la camara cuando esta es cambiada o desconectada por diferntes
            razones.

            Returns
            -------
            ret : Bool
                Devuelve True si se detecta algun frame y false en caso contrario.
            frame : Array
                Guarda la matriz de NxM del frame en un arreglo.
            camara : Objeto
                Genera el objeto Camara para acceder a sus metodos get y set.

            '''
        camara = cv2.VideoCapture("DJI_20230616175722_0007_D.JPG")
        camara, h, w = self.CamaraSettings(camara)
        ret, frame = camara.read()
        return ret, frame, camara

    '''Funcion utilizada para configuración inicial de camara (parametros criticos)'''

    def CamaraSettings(self,camara) :
        '''
        Descripcion
        -------
        Configuarcion de la camara

        Parameters
        ----------
        camara : Objeto
            Obtiene el objeto camara como parametro para trabajar con el.

        Returns
        -------
        camara : Objeto
            Es el objeto Camara para acceder a sus metodos get y set.
        h : double
            Altura del frame obtenido mediante el objeto camara
        w : double
            Ancho del frame obtenido mediante el objeto camara

        '''
        camara.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        camara.set(cv2.CAP_PROP_FOCUS, 100000)
        camara.set(cv2.CAP_PROP_FPS, 60)
        camara.set(cv2.CAP_PROP_FRAME_WIDTH, 800)#840
        camara.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)#400
        w = camara.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = camara.get(cv2.CAP_PROP_FRAME_HEIGHT)
        h = int(h)
        w = int(w)
        return camara, h, w

    '''Función utilizada para activar sistema de corte via MODBUS TCP/IP'''

    def Corte(self,BroccoliSize, OnOff) :
        '''
        Descripcion
        -------
        Activacion del sistema de corte al cruzar el bigote de gato
        y ser medido de manera adecuada

        Parameters
        ----------
        BroccoliSize : double
            Tamaño del brocoli actual
        Returns
        -------
        None.0

        '''
        if OnOff == "si" :
            self.Trigger(6,1)
            self.Trigger(6,0)
            

    '''Función utilizada para establecer los ajustes de Configuración de YoloV5'''

    def AjustesYoloV5(self,model, names) :
        '''
        Descripcion
        -------
        Opciones de ajustar de manera manual los parametros y valiores del
        modelo de IA o dejar los ajustes pre-establecidos

        Parameters
        ----------
        model : Objeto
            Modelo de IA utilizado para acceder a sus metodos set.
        names : List
            Lista de nombres de los objetos identificables por la IA.

        Returns
        -------
        None.

        '''
        SettingsFlag = True
        print("Ajustes de Parametros... Si no sabe lo que esta realizando por favor escriba 'no'")
        Ajustes = input("Desea Ajustar parametros a YoloV5(si/no): ")
        Respuesta = Ajustes.casefold()

        while 1 :
            if Respuesta == "si" :
                print("Ajustes".center(50, "*"))
                model.conf = float(input("Confidence Threshold(0-1): "))
                model.iou = float(input("NMS IoU threshold (0-1): "))
                model.classes = tuple(map(int, input(f"Clases Seleccionadas (0-{len(names)}): ").split(",")))

                model.agnostic = bool(input("NMS class-agnostic: "))  # NMS class-agnostic
                model.multi_label = bool(input("NMS multiple labels per box: "))  # NMS multiple labels per box
                model.max_det = int(
                    input("Maximum number of detections per image: "))  # maximum number of detections per image
                model.amp = bool(
                    input("Automatic Mixed Precision (AMP) inference: "))  # Automatic Mixed Precision (AMP) inference
                model.to("cuda")
                print("\n")
                break

            if Respuesta == "no" :
                print("Ajustes".center(50, "*"))
                print("Ajustes Predeterminados Seleccionados...\n")
                model.conf = 0.5  # confidence threshold (0-1)
                model.iou = 0.5  # NMS IoU threshold (0-1)
                model.classes = 0, 50
                model.agnostic = False  # NMS class-agnostic
                model.multi_label = False  # NMS multiple labels per box
                model.max_det = 100  # maximum number of detections per image
                model.amp = False  # Automatic Mixed Precision (AMP) inference
                model.to("cuda")
                print("\n")
                break
            Ajustes = input("Escribir \'si\' o \'no\': ")
            Respuesta = Ajustes.casefold()

    '''Función utilizada para habilitar/Desabilitar las funciones TCP/IP desde la configuración inicial con el fin de evitar problemas de conexión en caso de tener apagada la tarjeta de control'''

    def TCPToggleOnOff(self) :
        '''
        Descripcion
        -------
        Para evitar errores de conexion se establece si sera o no utilizado
        el protocolo de coumunicacion ModbusTCP

        Returns
        -------
        Respuesta : string

        '''
        ConfigToggle = input(
            "Desea Establecer La conexion ModbusTCP?(si/no)\nSi la placa PCB no tiene energia por favor escribir No: ")
        Respuesta = ConfigToggle.casefold()
        while 1 :
            if Respuesta == "si" or Respuesta == "no" :
                return Respuesta
            ConfigToggle = input("Escribir \'si\' o \'no\': ")
            Respuesta = ConfigToggle.casefold()
            '''
            else:
                while (Respuesta != "si" and Respuesta != "no"):
                    ConfigToggle = input("Escribir \'si\' o \'no\': ")
                    Respuesta = ConfigToggle.casefold()
             '''

    '''Establece de manera manual si las lineas mostradas / bigotes de gatos, seran en vertical u  (esto no determina si se utiliza x o y para el contador)'''

    def ConfiguracionLineas(self) :
        '''
        Descripcion
        -------
        Establece de manera manual si las lineas mostradas / bigotes de gatos, seran en vertical u horizontal
        mas no establece si se utiliza y1 o x1 como paramatero para aumentar el contador

        Returns
        -------
        Respuesta : String
            v --> lineas en Vertical
            h --> lineas en Horizontal
            otro --> pergunta de nuevo

        '''
        print("Escribir 'h' correra el programa para Brocolis\nEscribir 'v' correra el programa para personas")
        ConfigVisual = input("Desea lineas Verticales u Horizontales (v/h): ")
        Respuesta = ConfigVisual.casefold()
        while 1 :
            if Respuesta == "v" or Respuesta == "h" :
                return Respuesta
            ConfigVisual = input("Escribir \'v\' o \'h\': ")
            Respuesta = ConfigVisual.casefold()

    '''Esta función hace parpadear los leds de estado en la tarjeta de control de cada sistema de visión, para indicar conexión modbus exitosa'''

    def TestComunicacionModbusTPCIP(self,OnOff) :
        '''
        Descripcion
        -------
        Muestra de manera visual, encencdiendo y apagando n veces los leds,
        en el chasis cuando se ha creado y configurado de manera correcta el objeto Camara

        Parameters
        ----------
        Hace o no uso de los laseres dependiendo de la decision del Usuario

        Returns
        -------
        None.

        '''
        if OnOff == "si" :
            '''
            for i in range(0, 5):
                modbusClient = ModbusClient('192.168.100.10', 502)
                modbusClient.connect()
                modbusClient.write_single_coil(6, 1)
                modbusClient.close()
                time.sleep(0.5)
                modbusClient = ModbusClient('192.168.100.10', 502)
                modbusClient.connect()
                modbusClient.write_single_coil(6, 0)
                modbusClient.close()
                time.sleep(0.5)

            modbusClient = ModbusClient('192.168.100.10', 502)
            modbusClient.connect()
            modbusClient.write_single_coil(3, 1)
            modbusClient.close()
            '''
            self.Trigger(1,1)

            print("no jala")
        else :
            print("Conexion Modbus No Iniciada")
        # InicializacionForzadaProcesos()

    '''Conexión con SERVER SQL'''

    def EstablecerConexionSERVER_SQL(self) :
        # server = '192.168.100.11' antiguo
        # server = '192.168.0.124'
        server = 'LOCALHOST'
        baseData = 'BrocoliHarvesterIoT'
        user = 'sa'
        password = 'P1rqu2D23nn4v1c34n@'
        print("Servidor".center(50, "*"))
        print("Iniciando Conexion con Servidor...")
        conexion = Conexion(server, baseData, user, password)
        print("Conexion con Servidor Establecida...")
        return conexion

    def InicializacionForzadaProcesos(self):
        print("jala")
        for i in range(1,10):  
         self.Trigger(i,0)
         print("1")
       
            
            

    def corte_con_pilas(self,velocidad, pilaT, distancia) :
        if pilaT[len(pilaT) - 1] != 0 :
            senal = (pilaT[1] + 1 - (0.892 * velocidad))

            if senal < distancia and distancia > 0.80 :
                print("corar brocoli")
                # solo cuando este conectado
                '''
                Trigger(6,1)
               '''
                pilaT.pop(1)
                return pilaT
        return pilaT
    """ Funcion que controla los trigger de la comunicacion del arduino"""
    def Trigger(self,activacion,accion):
        modbusClient = ModbusClient('192.168.100.10', 502)
        modbusClient.connect()
        modbusClient.write_single_coil(activacion, accion)
        modbusClient.close()

    def Velocidad_Prueba(self,puntosXY, tiempo, distancia) :
        dist = 0
        veloc = 0
        temp = 0
        dist = math.sqrt((puntosXY[1][0] - puntosXY[0][0]) ** 2 + (puntosXY[1][1] - puntosXY[0][1]) ** 2)
        temp = distancia * 0.25 / 100
        veloc = dist * temp / tiempo
        if veloc >= 1 :
            print("distancia:", dist * temp, "cm")
            print("Velocidad del objeto", veloc, "cm/s")
            return veloc
        return 0

    def tamano_brocoli(self,diamtro_pixeles, distancia) :
        tam = 0
        dista1 = diamtro_pixeles[1] - diamtro_pixeles[0]
        dista2 = diamtro_pixeles[3] - diamtro_pixeles[2]
        tamPix = distancia * 0.25 / 100
        tam = tamPix * dista1
        tam2 = tamPix * dista2
        return tam, tam2
        # dist=math.sqrt((puntosXY[1][0]-puntosXY[0][0])**2-(puntosXY[1][1]-puntosXY[0][1]**2)

    def velocidad(self):
        var_velocidad = 2
        var_velocidad = var_velocidad * 1000 / 3600
        # se prentede octener la velocidad de la camara
        return var_velocidad

    def distancias(self,tam, mean, depth) :
        vec = []
        contj = tam
        while contj >= -tam :
            conti = tam
            while conti >= -tam :
                vec.append(depth[mean[0] + conti, mean[1] + contj])
                conti -= 1
            contj -= 1
        return vec

    def Velocidad(self,ruido, accel, accelA, sumaAnte, tiempo) :
        tiempo = tiempo - time.time()
        suma = (sumaAnte + accel + accelA - 2 * ruido) * (tiempo / 2)
        vel = abs((suma - sumaAnte) / tiempo)
        return vel, accel, suma, time.time()

    def ruido(self) :
        dc = DepthCamera()
        tiempo = time.time()
        ruido = []
        while (time.time() - tiempo) <= 2 :
            ret2, depth_frame, color_frame, accel = dc.get_frame()
            ruido.append(accel[0])
        ruido = sum(ruido) / len(ruido)
        return ruido

    def busqueda(self) :
        cont = 1
        try :
            while True :
                ar = open(f'D:\Documentos brocolis\prueba_{cont}.txt')
                cont += 1
                ar.close()
        except Exception :
            pass
        return cont

    '''Programa Principal'''

    def main(self,BroccoliAcceptedSizes) :
        '''
        Descripcion
        -------
        Codigo principal donde se muestra el video y se ejecutan las acciones condicionales

        Parameters
        ----------
        BroccoliAcceptedSizes : Tuple
            Rango de Tamaños aceptados por el productor.

        Variables
        ----------
        BroccoliSizeMin, BroccoliSizeMax --> Tamaño maximo y minimo del brocoli
        BroccoliSize, NumLasers = None, 2
        x1, x2, y1, y2, xmean, ymean --> Valores obtenidos de YoloV5
        showImageFlag --> Bandera para mostrar o no la ultima imagen analizada correctamente
        ------------------------------------------------------------------------------------
        Debugear codigo de muestra de la toma decision del contador para entender estas
        variables

        NumberOfSlots --> Tamaño del Buffer para la logica del contador
        bAnt = NumberOfSlots * [0]
        bAct = NumberOfSlots * [0]
        listaAct = NumberOfSlots * [0]
        listaAnt = NumberOfSlots * [0]
        ------------------------------------------------------------------------------------
        AnalysisPictureFlag --> Bandera que indica cuando y que se debe iniciar el proceso de
                                analisis del BoundingBox
        BroccolisCounter --> Contador de Brocolis
        tiempos --> Lista de tiempos que le toma a todo proceso por cada frame
        endVideo --> Termina el ciclo While dando lugar al cierre del Porgrama Principal
        h --> altura del frame
        h2 --> mitad de la altura del frame
        w --> ancho del frame
        w2 --> mitad del ancho del frame
        BigoteDeGato0 --> Valor que conforma el rango de deteccion de un
                                                 Brocoli Nuevo
        BigoteDeGato1_unOctavo --> Linea de un octavo del ancho o alto del frame que sera el primer
                          condicional para detectar y analizar un brocoli
        intentosReconexion --> Numero de intentos que lleva el proceso de reconexion antes de fallar
        intentosDeReconexionAceptados --> Numero de intentos Permitidos antes de terminar el programa
        CamaraOpenTimeFlag --> Bandera para solo mostrar una vez el tiempo de inicio del video
        TCP_toggle_onoff -->


        Variables al cruzar el BoundingBox de un Octavo
        ----------

        imagenBoundingBox --> Espacio del bounding box que sera analizada
        distY --> Tamaño del alto del BoundingBox
        distX --> Tamaño del ancho del BoundingBox
        BroccoliSize -->Tamaño del brocoli
        distX2Cm --> Tamaño del ancho del BoundingBox en cm
        distY2Cm --> Tamaño del ancho del BoundingBox en cm
        TamañoDimCmY --> Tamaño del brocoli en funcion del alto del BoundingBox
        TamañoDimCmX --> Tamaño del brocoli en funcion del ancho del BoundingBox
        ImageStatus --> regresa 0 si en la imagen analizada no se encontraron Laseres o hubo errores
                        y 1 si todo en orden


        cutThread --> MultiHilo para el sistema de corte
        lastImageTaken --> Imaegn del ultimo brocoli analizado correctamente


        Returns
        -------
        None.

        '''
        print("Confirmacion Tamaño de Brocolis: ", BroccoliAcceptedSizes)
        print("Laseres Encendidos")
        print("Iniciando Programa".center(50, "*"))
        print(
            f"Setup complete. Using torch {torch.__version__} ({torch.cuda.get_device_properties(0).name if torch.cuda.is_available() else 'CPU'})")
        CameraOpen_StartingTime = time.time()

        #dc = DepthCamera()  # colocar en mejor lugar
        camara = cv2.VideoCapture("Video.MP4")
        camara, h, w = self.CamaraSettings(camara)

        # print(camara.read())
        # print(camara.getBackendName())

        TCP_toggle_onoff = self.TCPToggleOnOff()
        self.TestComunicacionModbusTPCIP(TCP_toggle_onoff)
        print("Ancho: ", w, "Alto: ", h)

        # Variables y Constantes iniciadas
        BroccoliSizeMin, BroccoliSizeMax = BroccoliAcceptedSizes
        BroccoliSize, NumLasers = None, 2
        x1 = x2 = y1 = y2 = xmean = ymean = None
        showImageFlag = False
        NumberOfSlots = 20
        bAnt = NumberOfSlots * [0]
        bAct = NumberOfSlots * [0]
        listaAct = listaAnt = NumberOfSlots * [0]
        AnalysisPictureFlag = 0
        BroccolisCounter = 0
        tiempos = []
        endVideo = False
        h2 = int(h / 2)
        w2 = int(w / 2)
        BigoteDeGato0 = 100
        BigoteDeGato1_unOctavo = int(h / 8)
        intentosReconexion = 0
        intentosDeReconexionAceptados = 5
        CamaraOpenTimeFlag = True

        # area_puntos=np.array([int(2*w/3),0],[int(w/3),0],[int(w/3),h],[int(2*w/3),h])
        PilaTiempo = [0]
        distance = []
        tiempo_detecion = 0
        tiempo_distancia = 0
        bandera = True
        puntos = []
        veloc = 0
        tamano_RealX = 0
        tamano_RealY = 0
        bro = 20 * [0]
        pos = 0
        accel_ant = sumaA = vel = 0
        tiempo = 0
        cont = self.busqueda()
        distancia = 0
        Archivo = open(f'D:\Documentos brocolis\prueba_{cont}.txt', "w")
        # Setup Modelo YOLOV5
        # fileXml = FileXMl(r"C:\Users\admin\Desktop\PruebasCodigos\PathPrueba\\", "Prueba1111123.xml")
        names = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
                 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
                 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase',
                 'frisbee',
                 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
                 'surfboard',
                 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
                 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
                 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard',
                 'cell phone',
                 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
                 'teddy bear',
                 'hair drier', 'toothbrush']

        # model = torch.hub.load('ultralytics/yolov5', 'custom', 'yolov5m-seg.pt')
        # model = torch.hub.load('WongKinYiu/yolov7','custom', 'yolov7.pt')
        # model = torch.hub.load('ultralytics/yolov5', 'custom', 'yolov5m.engine', force_reload=True)
        model = torch.hub.load(r'yolov5-master', 'custom', 'yolov5m6.pt', source='local')
        # model = torch.hub.load(r'yolov7-main', 'custom', 'yolov7.pt',source='local')
        self.AjustesYoloV5(model, names)
        visionConfig = self.ConfiguracionLineas()
        # Escritor de Video
        tiempo = time.time()
        vec = []
        '''
        while (time.time() - tiempo) <= 2 :
            accel = dc.accel_data()
            vec.append(accel[0])
        fondo = sum(vec) / len(vec)
        '''
        tiempo = 0
        Archivo.write(f'{datetime.now().strftime("%d/%b/%Y")}  {datetime.now().strftime("%H:%M %p")}\n')
        Archivo.write('*********************************************************************************\n')
        Videoresult = cv2.VideoWriter(r'Muestra.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (int(w), int(h)))
        if camara.isOpened() :
            while 1 :
                distancia = distancia + vel * (time.time() - tiempo)
                TimePerFrame_StartingPoint = time.time()
                FrameTimerStarting = time.time()
                if CamaraOpenTimeFlag :  # sirve para poder mostrar el tiempo que tardado en la configuracion de todas sus parates
                    CameraOpen_EndingTime = time.time()
                    print("Tiempo de Inicio: ", CameraOpen_EndingTime - CameraOpen_StartingTime)
                    CamaraOpenTimeFlag = False

                # Confirmacion de lectura del video
                ret, frame = camara.read()
                #------ret2, depth_frame, color_frame, accel = dc.get_frame()


                """**************Velocidad********************"""
                #------vel, accel_ant, sumA, tiempo = self.Velocidad(fondo, accel[0], accel_ant, sumaA, tiempo)

                # print(vel)

                """*******************************************"""
                while ret == False :
                    if intentosReconexion < intentosDeReconexionAceptados :
                        intentosReconexion += 1
                        print(f"Intentando Reconexion...\nIntento #{intentosReconexion}/5")
                        ret, frame, camara = self.ConexionCamara()
                        if ret == True :
                            print("Conexion Restablecida")
                            intentosReconexion = 0
                        time.sleep(5)
                    else :
                        print("Numero de Intentos Excedidos, Cerrando Programa...")
                        endVideo = True
                        break
                if endVideo == True :
                    break

                # Lector de FPS
                fps = camara.get(cv2.CAP_PROP_FPS)
                # print(fps)

                # Correccion de imagen
                frame = cv2.flip(frame, 1)
                # Ingreso del frame al modelo para analizar
                results = model(frame, size=640)
                # Obtencion de nombre y posicion del objeto detectado
                labels = cord = results.xyxyn[0][:, :-1]
                classes = labels.tolist()
                infoPandas = results.pandas().xyxy[0]
                # Numero de objetos detectados en el video
                objetosEnVideo = len(cord)
                # Muestra los BBs en el frame actual
                Bb = np.squeeze(results.render())
                # ***Ajustes de Lineas Visuales de los Bigotes de Gato***

                if showImageFlag == True :
                    frame[0 :200, 0 :200] = lastImageTaken

                # cv2.putText(frame, f'Ultimo Tamano: {BroccoliSize}', (w-280, h-40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 255, 124), 2)
                # if BroccoliSize == None:
                # cv2.putText(frame, f'Medido: ___cm', (500, h-28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 255, 124), 2)
                # else:
                # cv2.putText(frame, f'Medido: {BroccoliSize:.2f}cm', (500, h-28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 255, 124), 2)

                # Muestra de Numero de objetos detectados en el Frame
                # cv2.putText(frame, f'No. de Objetos: {objetosEnVideo}', (w-340, h-28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 255, 124), 2)
                cv2.putText(frame, f"Brocolis Contados:{BroccolisCounter}", (w - 280, h - 68),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 255, 124), 2)
                cv2.putText(frame, f'Fecha: {datetime.now().strftime("%d/%b/%Y")}', (10, h - 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 255, 124), 2)
                cv2.putText(frame, f'Hora: {datetime.now().strftime("%H:%M %p")}', (10, h - 68),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 255, 124), 2)
                # cv2.putText(frame, f'Tamano: {tamano_RealX:.2f}', (w-280, h-40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 255, 124), 2)

                if visionConfig == "h" :
                    cv2.line(frame, (0, h2), (w, h2), (255, 0, 0), 1)
                    # cv2.line(frame, (202, 202), (202, 0), (255, 0, 0), 5)
                    # cv2.line(frame, (202,202), (0,202), (255, 0, 0), 5)

                if visionConfig == "v" :
                    # cv2.line(frame, (w-BigoteDeGato0, h), (w- BigoteDeGato0, 0), (200, 255, 124), 2)
                    cv2.line(frame, (int(2 * w / 3), h), (int(2 * w / 3), 0), (255, 0, 0), 1)
                    cv2.line(frame, (int(w / 3), h), (int(w / 3), 0), (255, 0, 0), 1)
                    cv2.line(frame, (0, int(h / 2)), (w, h2), (0, 0, 220), 1)
                    cv2.line(frame, (w2, h), (w2, 0), (0, 0, 220), 1)
                    # cv2.line(frame, (w-BigoteDeGato1_unOctavo, h), (w- BigoteDeGato1_unOctavo, 0), (255, 0, 0), 2)

                # distance = depth_frame[w2,h2]#centro
                # cv2.circle(frame, (w2,h2), 3, (0, 0, 255), 1)
                # cv2.putText(frame, format(distance), (w2,h2 - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

                for j in range(0, objetosEnVideo) :
                    # ***Obtencion de datos del BoundingBox***
                    x1 = int(cord[j][0] * w)
                    y1 = int(cord[j][1] * h)
                    x2 = int(cord[j][2] * w)
                    y2 = int(cord[j][3] * h)
                    xmean = int((x1 + x2) / 2)
                    ymean = int((y1 + y2) / 2)

                    # Toma de imagen BBox y obtencion de Datos del Brocoli
                    if AnalysisPictureFlag == -1 :

                        Detected = datetime.now()
                        imagenBoundingBox = frame[y1 :y2, x1 :x2]
                        distY = abs(y1 - y2)
                        distX = abs(x1 - x2)
                        timeStartConversion = datetime.now()
                        BroccoliSize, distX2Cm, distY2Cm, TamañoDimCmY, TamañoDimCmX, ImageStatus = self.AnalizadorDiametro(
                            imagenBoundingBox, distY, distX)
                        timeEndConversion = datetime.now()
                        '''
                            Thread  es un proceso para hacer procesamiento en paralelo
                        '''
                        cutThread = Thread(target=self.Corte, args=(BroccoliSize, TCP_toggle_onoff))
                        cutThread.daemon = True
                        cutThread.start()

                        tProcess = time.time()
                        AnalysisPictureFlag = 0
                        lastImageTaken = cv2.resize(imagenBoundingBox, (200, 200))
                        # harvestId = Lectura(conexion)

                        # EscrituraServidor.EscrituraTasksNew(conexion, (1,datetime.now(),distX/2,distY/2,distX,distY,datetime.now(),1,1,1))
                        # EscrituraServidor.EscrituraObstacles(conexion, (harvestId,1,datetime.now(),1,1,datetime.now(),distX2Cm,distY2Cm,1,1,1))
                        if ImageStatus == 0 :
                            cv2.imwrite(fr"Imagenes\Brocoli_{BroccolisCounter}_Error.jpg", imagenBoundingBox)
                            showImageFlag = False
                            # EscrituraServidor.EscrituraResults(conexion, (timeStartConversion, distX2Cm / 2, distY2Cm / 2, TamañoDimCmX, TamañoDimCmY, timeEndConversion, 1,1, 1, 0, 1, 1))
                        if ImageStatus == 1 :
                            # EscrituraServidor.EscrituraResults(conexion, (timeStartConversion, distX2Cm / 2, distY2Cm / 2, TamañoDimCmX, TamañoDimCmY, timeEndConversion, 1,1, 1, 0, 1, 1))
                            showImageFlag = True
                            cv2.imwrite(fr"Imagenes\Brocoli_{BroccolisCounter}.jpg", imagenBoundingBox)

                    # if x2 < w-BigoteDeGato0:
                    #     if x2 > w-BigoteDeGato1_unOctavo:
                    #         bAct[j] = 1
                    # if y1 > BigoteDeGato1_unOctavo:
                    #     if y1 < BigoteDeGato1_unOctavo + BigoteDeGato0:
                    #         bAct[j] = 1no
                    
                    pixeles=x2-x1
                    #cv2.putText(frame, "{}pixeles".format(pixeles), (xmean, ymean - 5), cv2.FONT_HERSHEY_PLAIN, 1,(0, 0, 0), 2)

                      
                    if y2 < 480 and x2 < 848 :
                       # distance = self.distancias(5, (ymean, xmean), depth_frame)
                        '''
                        distance.append(depth_frame[ymean,xmean])
                        distance.append(depth_frame[ymean+5,xmean])
                        distance.append(depth_frame[ymean-5,xmean])
                        distance.append(depth_frame[ymean,xmean+5])
                        distance.append(depth_frame[ymean,xmean-5])
                        distance.append(depth_frame[ymean+5,xmean+5])
                        distance.append(depth_frame[ymean+5,xmean-5])
                        distance.append(depth_frame[ymean-5,xmean+5])
                        distance.append(depth_frame[ymean-5,xmean-5])
                        '''
                        # print(distance)
                        promedio = 0
                        cont = 0
                        # sacar el promedio de la distancia
                        ''' ------------
                        for i in range(0, len(distance)) :
                            if distance[i] != 0 :
                                cont += 1
                                promedio += distance[i]
                        '''
                        
                        
                          
                        
                        
                        
                        if cont == 0 :
                            tamano_RealX = tamano_RealY = 0
                        else :
                            promedio = int((promedio / cont) / 10)
                            tamano_RealX, tamano_RealY = self.tamano_brocoli(([x1, x2, y1, y2]), promedio)

                       
                        #cv2.putText(frame, "{}cm".format(promedio), (x2, y2 - 20), cv2.FONT_HERSHEY_PLAIN, 1,(255, 255, 255), 2)
                        # cv2.putText(frame, "tam:{:.2f}".format(tamano_RealX), (x2, y2), cv2.FONT_HERSHEY_PLAIN, 1,
                        #            (0, 0, 255), 1)
                        # cv2.putText(frame, "{}cm/s".format(int(veloc)), (x2, y1), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0,255), )
                        distance.clear()

                        # --------------medicion de tiempo
                        if bandera :
                            deteccion = time.time()
                            puntos.append([xmean, ymean])
                            # print(puntos)
                            bandera = False
                        if ((time.time() - deteccion)) >= 1 and j == 0 :
                            puntos.append([xmean, ymean])
                            # print(puntos)
                            # print(promedio)
                            veloc = self.Velocidad_Prueba(puntos, (time.time() - deteccion), promedio)
                            puntos.clear()
                            bandera = True
                    # area=np.array([[282,0],[564,0],[282,480],[564,480]],np.int32)
                    # limite=cv2.pointPolygonTestno(area,(xmean,ymean),False)
                    # print(limite)

                    if tamano_RealX > BroccoliSizeMin and tamano_RealX <= BroccoliSizeMax :
                        cv2.ellipse(frame, (xmean, ymean), (int((x2 - x1) / 2), int((y2 - y1) / 2)), 0, 0, 360,
                                    (60, 255, 51), 2)
                        ban = True
                        # bro[j]=[tamano_RealX,tamano_RealY]
                    else :
                        cv2.ellipse(frame, (xmean, ymean), (int((x2 - x1) / 2), int((y2 - y1) / 2)), 0, 0, 360,
                                    (0, 0, 255), 2)
                        ban = False
                    bro[j] = [tamano_RealX, tamano_RealY]
                    cv2.circle(frame, (xmean, ymean), 3, (0, 0, 255), -1)
                    cv2.circle(frame, (x2, ymean), 3, (0, 0, 255), -1)
                    cv2.circle(frame, (xmean, y1), 3, (0, 0, 255), -1)
                    cv2.circle(frame, (x1, ymean), 3, (0, 0, 255), -1)
                    cv2.circle(frame, (xmean, y2), 3, (0, 0, 255), -1)

                    if ymean < h2 and ymean < h2 + 50 :
                        # if y1 < h:
                        if x1 > (w / 3) and x2 < (2 * w / 3 - 1) :
                            # if x2<(2*w/3-1):
                            bAct[j] = 1
                            pos = j
                    else :
                        bAct[j] = 0
                    # ***Escritura de Datos y Circulos en Pantalla***
                    # cv2.putText(frame, f"Persona:{j + 1}", (x1, y2-40),cv2.FONT_HERSHEY_COMPLEX, 0.5, (200, 255, 124), 2)
                    # cv2.putText(frame, f"Objeto: {names[int(classes[j])]}: x: {cord[j][0]} y: {cord[j][1]}\nx: {cord[j][2]} y: {cord[j][3]}", (20, (j+1)*20), cv2.FONT_HERSHEY_SIMPLEX, .5, (200, 255, 124), 2)
                    # cv2.circle(frame, (xmean, y2), 3, (0, 0, 255), 4)
                    # cv2.circle(frame, (xmean, ymean), 3, (0, 0, 255), 4)

                    # ************************************************

                # **************************************************************************************************************************************************************
                # ***Metodologia para contar Brocolis al pasar el bigote de gatos***
                suma = sum(bAct) - sum(bAnt)
                if suma > 0 :
                    dateOfDetection = datetime.now()
                    AnalysisPictureFlag = 1
                    BroccolisCounter += 1
                    idBroc = BroccolisCounter

                    PilaTiempo.append(distancia)
                    Archivo.write(
                        f"Brocoli: {BroccolisCounter}  tamaño x:{bro[pos][0]:.2f}  tamaño Y: {bro[pos][1]:.2f}\n")
                bAnt = bAct.copy()
                # for co in range(0, NumberOfSlots):
                #   bAnt[co] = bAct[co]
                # *******************************************************************************************

                # Window PopUp
                cv2.namedWindow("CamaraLinea", cv2.WINDOW_NORMAL)
                cv2.imshow("CamaraLinea", frame)
                Videoresult.write(frame)

                # ***Datos de Medidas de Dispersion***
                if cv2.waitKey(1) == ord("s") :
                    print("Tiempos de Procesamiento".center(50, "*"))
                    print("Valor MAX: ", max(tiempos))
                    print("Valor MIN: ", min(tiempos))
                    print("Rango: ", max(tiempos) - min(tiempos))
                    print("Promedio: ", np.mean(tiempos))
                    print("Desviacion Standar: ", np.std(tiempos))
                    print("Varianza: ", np.var(tiempos))
                    # print(tiempos)
                    break
                PilaTiempo = self.corte_con_pilas(vel, PilaTiempo, distancia)
                # tiempo_detecion=corte_prueba(velocidad(), tiempo_detecion, False)
                # Tiempos de analisis por cada Frame
                TimePerFrame_EndingPoint = time.time()
                timeProcessing = TimePerFrame_EndingPoint - TimePerFrame_StartingPoint
                # print(timeProcessing)
                tiempos.append(timeProcessing)
                # print(f"Tiempo de analisis: {timeProcessing}, Matriz de datos analizados: {framePrueba.shape}, Matriz de datos analizados COMPLETOS: {frame.shape}, Resolucion de la camara: {h,w}")
                # InicializacionForzadaProcesos()

            Archivo.close()

            camara.release()
            Videoresult.release()
            #dc.release()
            cv2.destroyAllWindows()


        else :
            print("Conectar Cámara para arranque")

obj=Vision()
'''Esta condicional inicializa el main() con los parametros de trabajo(rango de tamaño de cabezas) definido manualmente en codigo o recuperado de la base de datos'''        
if __name__ == "__main__": 
    
    #x = 10,13 #forzar manualmente valores de tamaño de cabezas de brócoli   
    #main(x)   #forzar la ejecución de main con valores de tamaño de cabezas de brócoli forzados en codigo 
    '''Obtención de datos de configuración de tamaños de brócoli a cosechar y estatus de carro activado desde el SERVER'''
    conexion_=obj.EstablecerConexionSERVER_SQL();
    try :
        with conexion_ :
            with conexion_.cursor() as cursor :
                cursor = conexion_.cursor()
                #Se declara sentencia como una string que contiene la instrucción SQL a ejecutar
                sentencia = f'SELECT TOP 1 * FROM BroccoliHarvesterRowsByHarvest ORDER BY BroccoliHarvesterRowsByHarvestId DESC' 
                #con curse.exectute se ejecuta la instrución SQL
                cursor.execute(sentencia)
                #Se  guarda en la lista registro los valores obtenidos  de SQL
                registro = list(cursor.fetchone())
                #se asignan a BroccoliSizeRange los valores de tamaño maximo y minimo que se obtienen de la Lista registro
                BroccoliSizeRange = registro[9 :11]
                print('Rango de Brocolis: ', BroccoliSizeRange)
                #obj.InicializacionForzadaProcesos() #solo funciona si hay conexion modbus activa
                print("salio")
                obj.main(BroccoliSizeRange)
                if registro[4] == True :
                    print("Carrito 1 Activado")
    except Exception as e :
        print("ErrorOutMain: ", e)
    finally :
        cursor.close()
    