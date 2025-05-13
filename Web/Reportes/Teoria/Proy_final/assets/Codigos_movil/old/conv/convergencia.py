#LIBRERIAS
import cv2
import numpy as np
 
#------INICIALIZACION CAMARA------
resolucionx=1280
resoluciony=720

camara=cam.initialize(0,resolucionx,resoluciony)
texto_titulo=""

#------INICIALIZACION ROBOT------
print("Conectando Robot 0.....")
robot0=Bt.connect(MAC)
print("Robot 0 OK")

robot_bt=robot0
robot=[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

#INICIALIZACION DE VALORES ROBOT FISICO 1
wd=0
wi=0
wmax=150 #Velocidad angular robot maxima
l=-185 #distancia del centro del robot al centro de las llantas (mm)
r=58 #radio llanta (mm)
L=120 #distancia entre centros de llantas (mm)

#ITERACIONES DE SIMULACION
tfin=100
i=0

#INICIALIZACION DE VARIABLES DE CONTROL
x,y,th,t,ex,ey,ux,uy = aux.inicializar_arreglos(tfin)

#------VALORES de ley de CONTROL------
xs=0 #valor esperado de x
ys=0 #valor y deseado
k=130 #ganancia del control

while (True):

    #-------BUSQUEDA DE QR´s------
    #print("Buscamos aruco")
    frame, points, ids =cam.buscar_Aruco(camara, resolucionx, resoluciony)
    #print(ids)

    if len(points)>0:
        robot=cam.buscar_robots(points, ids, robot)
        #print("robot0(x="+str(robot[0][0])+",y="+str(robot[0][1])+",th="+str(robot[0][2])+")")

        #------LEY DE CONTROL CONVERGENCIA------
        t[i]=i
        x[i]=robot[1][0]-(resolucionx/2) #Obtenemos valor X de QR robot 
        y[i]=robot[1][1]-(resoluciony/2) #Obtenemos valor Y de QR robot 
        th[i]=robot[1][2] #Obtenemos valor Th de robot 

        x[i]=x[i]+(l*np.cos(th[i]))
        y[i]=y[i]+(l*np.sin(th[i]))

        A=np.array([[np.cos(th[i]), -l*np.sin(th[i])],
                    [np.sin(th[i]), l*np.cos(th[i])]])

        ex[i],ey[i],ux[i],uy[i]=robotx.convergencia(x[i],y[i],xs,ys,k) #Calculo de errores y vector velocidad

        B=np.array([ux[i],uy[i]]) #Arreglo de Vector velocidad

        U=np.linalg.solve(A,B)
        #U=np.linalg.inv(A)*B  No sirve

        V=U[0] #Velocidad Lineal
        W=U[1]  #Velocidad Angular
         #print("v="+str(V)+"w="+str(W))

        #Convergencia
        # else:
        #     texto_titulo="CONVERGENCIA (auto)"
        #     color=(0, 0, 255)
        #     i=i+1

        #-------MODELO CINEMATICO ROBOT-------- 

        wd= (V/r)+((L*W)/(2*r))
        wi= (V/r)-((L*W)/(2*r))

        if(wd>wmax):
            wd=wmax
        elif(wd<-wmax):
            wd=-wmax

        if(wi>wmax):
            wi=wmax
        elif(wi<-wmax):
            wi=-wmax

        Bt.move(robot_bt,wd, wi)

    if (i>tfin):
        break


    #-------VENTANA DE CAMARA-------- 
    cam.dibujar_aruco(frame, points, ids, resolucionx,resoluciony)
    cam.draw_texto_titulo(frame, texto_titulo,color)
    cam.draw_punto(frame,"XS,YS",(0,0,255), xs+(int(resolucionx/2)), ys+(int(resoluciony/2)),resolucionx,resoluciony)
    cv2.imshow('Camara detector qr', frame) #Despliega la ventana 

    if cv2.waitKey(1) & 0xFF == 27: #Presiona esc para salir 
        break

#-------RUTINA DE CIERRE-------- 
Bt.move(robot_bt,0,0)  #Apagamos motores en Robot
Bt.disconnect(robot_bt) #Desconectamos Bluetooth
camara.release() #Liberamos Camara
cv2.destroyAllWindows() #Cerramos ventanas