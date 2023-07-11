# import random
import time
import cv2
import numpy as np
import math



# BsizeRandom = [x for x in range (0, 30)]

def AnalizadorDiametro(imagen, x, y) :
    try :
        distX = x
        distY = y
        Tamaño = 0

        # Declaracion de listas y variables
        listaXYContornosCentros = []
        DiccPitagorasP12P2 = {}
        listaDistanciaPitagoras = []
        diccPitagoras = {}
        diccPitagoras2_ApoyoVisual = {}
        i = 0
        distanciaConversionX = 0
        distanciaConversionY = 0
        disFinal = 0
        pitagoras = 0

        # Seleccion De rangos y kernel para los filtros
        colorBajo = np.array([0, 0, 247], np.uint8)
        colorAlto = np.array([179, 22, 255], np.uint8)
        kernel = np.ones((5, 5), np.uint8)  # aumentar el numero del kernes si hay demasiado ruido de puntos

        # Lectura de imagen y conversion binarizada
        imageRGB = imagen  # imagen recibida desde la CNN
        imageHSV = cv2.cvtColor(imageRGB, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(imageHSV)
        imagePuntos = cv2.inRange(imageHSV, colorBajo, colorAlto)

        # Comparativa(BORRAR)
        h, w, _ = imageRGB.shape
        # print(h == y, w == x)

        # Limpieza de puntos ->> Posibles Modificaciones para mejorar la limpieza de puntos
        # opening = cv2.morphologyEx(imagePuntos, cv2.MORPH_OPEN, kernel)
        dilation = cv2.dilate(imagePuntos, kernel, iterations=1)
        canny = cv2.Canny(dilation, 0, 255)
        cnts, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        alto, ancho, _ = imageRGB.shape
        puntoRefx = int(ancho / 2)
        puntoRefy = int(alto / 2)
        cv2.circle(canny, (puntoRefx, puntoRefy), 3, (255, 255, 255), 0)

        # Localizacion del centro de los lasers
        for cnt in cnts :
            area = cv2.contourArea(cnt)
            if ((area > 0) & (area < 300)) :
                M = cv2.moments(cnt)
                if M["m00"] == 0 : M["m00"] = 1
                # Centros X,Y de los contornos
                x = int(M["m10"] / M["m00"])
                y = int(M["m01"] / M["m00"])
                cv2.circle(canny, (x, y), 1, (255, 255, 255), -1)
                listaXYContornosCentros.append([x, y])  # Lista [[x,y]] que guarda los centros de cada laser encontrado

        # Obtencion de la distancia X y Y con respecto al punto de referencia para caluclar su hipotenusa
        for xy in range(0, len(listaXYContornosCentros)) :
            distx = abs(puntoRefx - listaXYContornosCentros[xy][0])
            disty = abs(puntoRefy - listaXYContornosCentros[xy][1])
            pitagoras = math.sqrt(distx ** 2 + disty ** 2)
            listaDistanciaPitagoras.append(pitagoras)
            diccPitagoras[f'{pitagoras}'] = [listaXYContornosCentros[xy][0],
                                             listaXYContornosCentros[xy][
                                                 1]]  # Relacion Hipotenusa : Centro de Contornos

        valMin1 = min(listaDistanciaPitagoras)  # Valor mas pequeños dentro de la lista de hipotenusas
        listaDistanciaPitagoras.clear()
        listaXYContornosCentros.remove(diccPitagoras[f'{valMin1}'])

        # print("Val mas cercano al centro: ", valMin1)

        listaDistanciaPitagoras.clear()  # Se elimina para buscar el siguiente valor mas pequeño
        # print(f'ValP1 = {diccPitagoras[f"{valMin1}"]}\nX: {diccPitagoras[f"{valMin1}"][0]}\nY: {diccPitagoras[f"{valMin1}"][1]}')

        for xy2 in range(0, len(listaXYContornosCentros)) :
            distx = abs(listaXYContornosCentros[xy2][0] - diccPitagoras[f"{valMin1}"][0])
            disty = abs(listaXYContornosCentros[xy2][1] - diccPitagoras[f"{valMin1}"][1])
            pitagoras = math.sqrt(distx ** 2 + disty ** 2)
            listaDistanciaPitagoras.append(pitagoras)
            DiccPitagorasP12P2[f'{pitagoras}'] = [listaXYContornosCentros[xy2][0], listaXYContornosCentros[xy2][
                1]]  # Relacion Hipotenusa : Centro de Contornos

        # print("Dicc 2, ", DiccPitagorasP12P2)
        valMin2 = min(listaDistanciaPitagoras)
        # print("Val mas cercano al primer laser encontrado: ", DiccPitagorasP12P2[f'{valMin2}'])

        # Distancia absoluta entre los centros de los valores obetnidos (x2-x1, y2-y1)
        distanciaConversionX = abs(diccPitagoras[f'{valMin1}'][0] - DiccPitagorasP12P2[f'{valMin2}'][0])
        distanciaConversionY = abs(diccPitagoras[f'{valMin1}'][1] - DiccPitagorasP12P2[f'{valMin2}'][1])
        distFinalPx = math.sqrt(
            distanciaConversionX ** 2 + distanciaConversionY ** 2)  # c = sqrt((x2-x1)^2 + (y2-y1)^2)

        cv2.line(canny, (diccPitagoras[f'{valMin1}'][0], diccPitagoras[f'{valMin1}'][1]), (puntoRefx, puntoRefy),
                 (255, 255, 255), 2)
        cv2.line(canny, (diccPitagoras[f'{valMin1}'][0], diccPitagoras[f'{valMin1}'][1]),
                 (DiccPitagorasP12P2[f'{valMin2}'][0], DiccPitagorasP12P2[f'{valMin2}'][1]), (255, 255, 255), 2)
        # cv2.line(canny, (diccPitagoras[f'{valMin1}'][0], diccPitagoras[f'{valMin1}'][1]), (diccPitagoras[f'{valMin1}'][0] - distanciaConversionX, diccPitagoras[f'{valMin1}'][1] + distanciaConversionY), (255,255,255), 2)

        distRealCmEntreLasers = 5.32
        FactorDeConversionPx2Cm = distRealCmEntreLasers / distFinalPx

        # Para la conversion final es factor*diametroDelBrocoliEnPx
        # print("Factor (In):{}\nNumero de Lasers: {} ".format(FactorDeConversionPx2Cm, numLasers))
        distX2Cm = FactorDeConversionPx2Cm * distX
        distY2Cm = FactorDeConversionPx2Cm * distY

        TamañoCmX = FactorDeConversionPx2Cm * distX
        TamañoCmY = FactorDeConversionPx2Cm * distY

        if distX > distY :
            Tamaño = TamañoCmX
        if distY >= distX :
            Tamaño = TamañoCmY
        # print("Factor de Conversion Px2Cm: ", FactorDeConversionPx2Cm)
        ImStatus = 1

        return Tamaño, distX2Cm, distY2Cm, TamañoCmY, TamañoCmX, ImStatus

    except Exception as E :
        ImStatus = 0
        # Tamaño = None
        TamañoCmX = 0
        TamañoCmY = 0
        distX2Cm = 0
        distY2Cm = 0
        Tamaño = None
        # print("Error, ", E)
        return Tamaño, distX2Cm, distY2Cm, TamañoCmY, TamañoCmX, ImStatus

# if __name__ == "__main__":
#     listaTiemposAnalisis = []
#     for i in tqdm(range(1000000)):
#         foto = cv2.imread(r"1.jpeg")
#         tAnalizador = time.time()
#         # print("Inicio")
#         Tamaño, distX2Cm, distY2Cm, TamañoCmY, TamañoCmX, ImStatus = AnalizadorDiametro(foto,random.randint(100,500),random.randint(100,500))
#         tAnalizador2 = time.time()
#         tFinal =  tAnalizador2-tAnalizador
#         # print("Tiempo de analisis: ", tFinal)
#         listaTiemposAnalisis.append(tFinal)
#     print("Tiempos de Procesamiento".center(50,"*"))
#     print("Valor MAX: ", max(listaTiemposAnalisis))
#     print("Valor MIN: ", min(listaTiemposAnalisis))
#     print("Rango: ", max(listaTiemposAnalisis)-min(listaTiemposAnalisis))
#     print("Promedio: ", np.mean(listaTiemposAnalisis))
#     print("Desviacion Standar: ", np.std(listaTiemposAnalisis))
#     print("Varianza: ", np.var(listaTiemposAnalisis))