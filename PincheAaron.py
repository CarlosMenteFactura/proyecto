from easymodbus.modbusClient import ModbusClient
class Conexion:
    
    def Trigger(self, activacion):
        try:
            modbusClient = ModbusClient('192.168.100.10', 502)
            modbusClient.connect()
            modbusClient.write_single_coil(activacion, 1)
          
            modbusClient.close()
            modbusClient = ModbusClient('192.168.100.10', 502)
            modbusClient.connect()
            modbusClient.write_single_coil(activacion, 0)
          
            modbusClient.close()
        except:
            print("aduino no detectado")
    def Activacion(self):
        print("************Escribe*************")
        while 1:
            dispositivo=int(input("Trigger a modificar \n"))
            self.Trigger(dispositivo)
            respuesta=input("seguir testeando no (n) \n")
            if respuesta=="n":
                break
if __name__=="__main__":
    obj=Conexion() 
    obj.Activacion()