import pyodbc as bd
def Conexion(server,baseData, user,password):
    conexion = bd.connect(driver='{SQL server}', host=server, database=baseData, user=user, password=password)
    return conexion