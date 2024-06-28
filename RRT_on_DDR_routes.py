from vispy import app
import sys
from vispy.scene import SceneCanvas
from vispy.scene.visuals import  Rectangle, Text, Polygon, Line
from vispy.color import Color
import numpy as np
from shapely.geometry import Polygon as Polygon_geometry
import random
import copy

white = Color("#ecf0f1")
gray = Color("#121212")
red = Color("#e74c3c")
blue = Color("#2980b9")
orange = Color("#e88834")
cyan = Color("#00FFFF")



#Variables
INF = float("inf")
tol_error = 30

#Funciones de apoyo
#########################################################################################################
def alpha(teta1, teta2):
  return min(abs(teta1-teta2),2*np.pi - abs(teta1-teta2))

def disntancia_estados(X,Y):
  return np.sqrt( (X[0]-Y[0])**2 + (X[1]-Y[1])**2 + alpha(X[2],Y[2])**2 )

def get_angulo_x(p1,p2): #Angulo de p1 a p2 respecto al eje X (regresa valores negativos tambien)
    return np.arctan2(p2[1] - p1[1], p2[0] - p1[0])

def Rotacion(point, theta):#Rota un vector point en un angulo theta respecto al origen
  R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
  return R@point

#Determina si hay colision con Poligon y el obstaculo
#Considera colision de aristas y vertices
def is_in_obstaculo(Poligon, obstaculo):
   return Poligon.intersects(obstaculo)

#Dados los valores de un estado, regresa una construccion del mismo
def create_estado(X,r,b):
    x = X[0]
    y = X[1]
    theta = X[2]

    #Consideramos un robot centrado en el origen, el cual rotaremos en un angulo teta, y luego lo trasladaremos al centro (x,y)
    """
    v1---v2
    |     |
    |  *  |
    |     |
    v3---v4
    """

    v1 = Rotacion([-r,b],theta) + [x,y]
    v2 = Rotacion([r,b],theta) + [x,y]
    v3 = Rotacion([-r,-b],theta) + [x,y]
    v4 = Rotacion([r,-b],theta) + [x,y]

    return [v1,v2,v4,v3,v1]

def is_on_limits(X,xlim,ylim):#Verificamos que el centro del estado se encuentre dentro de la pantalla
  if xlim[0] > X[0] or X[0] > xlim[1]:
    return False
  if ylim[0] > X[1] or X[1] > ylim[1]:
      return False

  return True

def is_free_crash(X,r,b,obstaculos, xlim, ylim):#Determinamos si el estado X esta en colision con algun obstaculo
    vertices_robot = create_estado(X,r,b)

    Robot = Polygon_geometry(vertices_robot)

    if is_on_limits(X,xlim,ylim) == False:
       return False

    for obstaculo in obstaculos:#Si llega a chocar con un obstaculo, regresamod False
       obstaculo_polygon = Polygon_geometry(obstaculo)
       if is_in_obstaculo(Robot,obstaculo_polygon):#Si el robot y el poligono se intersectan
          return False

    #Si no se intersecto con ningun obstaculo   
    return True

#Regresa una lista de los nodos de un grado desde nodo a padre segun las aristas E
def look_for_padre(padres,vertices, nodo, id):
    Road = [nodo]
    #print("\nIniciamos la busqueda del padre")
    id_hijo = copy.deepcopy(id)
    #print(f"Añadimos el indice {id_hijo}")
    while True:
        id_padre = padres[id_hijo]#Tomamos el idx del padre del nodo actual
        Road.append(vertices[id_padre])#Añadimos el vertice asociado
        #print(f"Añadimos el indice {id_padre}")
        id_hijo = copy.deepcopy(id_padre)#Volvemos al hijo el idx padre para la siguiente iteracion

        if id_hijo == 0:#Si llegamos a la cima rompemos
            break

    return Road


def create_rect(center, width, height):#Genera los vertices para un polígono rectangular según su centro, anchura y altura
   vertices = np.zeros((5,2))
   vertices[0] = center + np.array([-width/2,height/2])
   vertices[1] = center + np.array([width/2,height/2])
   vertices[2] = center + np.array([width/2,-height/2])
   vertices[3] = center + np.array([-width/2,-height/2])
   vertices[4] = vertices[0]
   return vertices

def draw_poligonos(obstaculos, COLOR):
   global view
   for obstaculo in obstaculos:
        poligono = Polygon(obstaculo, color=COLOR)
        view.add(poligono)
      
def look_for_padre(vertices, aristas, id_father, id_son):
    #Diccionario para guardar los padres de cada vertice
    padres = {}
    for edge in aristas:
        if edge[1] not in padres:
            padres[edge[1]] = []
        padres[edge[1]].append(edge[0])

    #Recorreremos los indices de las aristas para recorrer y añadir el camino desde goal a start
    Road = [vertices[id_son]]
    id_sink = padres[id_son]
    while id_sink[0] != id_father:
        Road.append(vertices[id_sink[0]])
        id_sink = padres[id_sink[0]]

    Road.append(vertices[id_sink[0]])#Añadimos al padre al final

    #Regresamos el camino invertido, desde start hacia goal
    return list(reversed(Road))
#########################################################################################################

#Declaracion de RRT
#########################################################################################################
class RRT:
    #Inicializamos el RRT con:
    #Inicio / goal / Dimensiones / Tiempo entre estados / Tiempo entre pasos / obsaculos / bordes del eje x / bordes del eje y
    def __init__(self, start, b, r, Dt, h, goal, obstaculos, xlim, ylim):
        self.start = start
        self.goal = goal
        self.obstaculos = obstaculos#Los obstaculos los consideraremos como conjuntos de vertices cerrados
        self.vertices = [start]
        self.edges = []
        self.xlim = xlim
        self.ylim = ylim
        self.b = b
        self.r = r
        self.Dt = Dt
        self.h = h
        #Supondremos que Dt es divisible entre h

    #Generemos un punto uniforme en los limites de la pantalla
    def Sample_Free(self):
        u = random.uniform(self.xlim[0], self.xlim[1])
        v = random.uniform(self.ylim[0], self.ylim[1])
        teta = random.uniform(0, 2*np.pi)
        return [u, v, teta]

    #Regresa el vertice del grafo mas cercano a un punto
    def Nearest(self, point):
        #Calcumos las distancias de los veritces al punto
        distances = [disntancia_estados(v,point) for v in self.vertices]
        #Obtenemos las distancia minima
        min_distance = min(distances)
        #Tomamos el indice que tiene tal distancia minima
        min_index = distances.index(min_distance)
        #Regresamos el vertice de tal indice
        return self.vertices[min_index], min_index

    #Generamos un camino hacia posibles x_new en base a combinaciones de wl y wr
    #Durante el camino de tamaños de paso h a lo largo de un tiempo Dt, veremos si no hay colisiones
    #La primer ruta exitosa en no tener colisiones se tomara como x_new y se unira al grafo
    def Steer(self, X, x_rand):
        Velocidades = [-1,1]
        #Inicializacion
        x_0 = X[0]
        y_0 = X[1]
        theta_0 = X[2]

        X_opt = copy.deepcopy(X)
        d_min = INF
        for wl in Velocidades:
            for wr in Velocidades:
                v = self.r*(wl+wr)/2#Velocidad lineal
                w = self.r*(wl-wr)/(2*b)#Velocidad angular

                count_time = 0
                x_k = x_0
                y_k = y_0
                theta_k = theta_0
                flag_found = True
                while count_time <= self.Dt:
                    x_k1 = x_k + self.h*(v * np.cos(theta_k))
                    y_k1 = y_k + self.h*(v * np.sin(theta_k))
                    theta_k1 = theta_k + self.h*w
                    #Si choca con algun obstaculo
                    if is_free_crash([x_k1,y_k1,theta_k1],r,b, obstaculos, self.xlim, self.ylim) == False:
                        flag_found = False
                        break#Saltamos a la siguiente combinacion
                    x_k = copy.deepcopy(x_k1)
                    y_k = copy.deepcopy(y_k1)
                    theta_k = copy.deepcopy(theta_k1)
                    count_time += self.h
                if flag_found:
                    #En este punto, el estado [x_k1,y_k1,theta_k1] es valido, queremos ver si es optimo
                    dist = disntancia_estados([x_k1,y_k1,theta_k1], x_rand)
                    if dist < d_min:
                        d_min = dist
                        X_opt = [x_k1,y_k1,theta_k1]
            
        if d_min < INF:
            return X_opt, True
        return X_opt, False

    #Añadimos un vertice al conjunto de vertices
    def add_vertex(self, new_vertex):
        self.vertices.append(new_vertex)


    #Añadimos una arista al conjunto de aristas
    def add_edge(self, from_vertex, to_vertex):
        self.edges.append((from_vertex, to_vertex))

    def plan(self, max_iter=15000, mu = tol_error):#mu es una distancia entre estaods minima para considerar a dos estados cercanos
        #Iteramos max_iter veces sobre el algoritmo RRT
        its = 1
        idx_vertex = 1
        for _ in range(max_iter):
            if its%500 == 0:
                print(f"Iteracion {its}/{max_iter}:\t|V|={len(self.vertices)}\t|E|={len(self.edges)}")
            x_rand = self.Sample_Free()
            x_nearest, idx = self.Nearest(x_rand)
            x_new, flag = self.Steer(x_nearest, x_rand)

            if flag:#Si se encontro un new_point con exito (No hubo colisiones en su proceso)
                self.add_vertex(x_new)
                self.add_edge(idx, idx_vertex)
                idx_vertex += 1
                #Si new_point puede percibir a goal en su rango mu
                if disntancia_estados(x_new, self.goal) <= mu:
                    #Incluimos a goal al grafo
                    self.add_vertex(self.goal)
                    self.add_edge(idx_vertex-1, idx_vertex)
                    return True
                
            its += 1
        return False

    def visualize(self, flag):
        global view

        vetices_robot = create_estado(self.start, r, b)
        vetices_goal = create_estado(self.goal, r, b)

        draw_poligonos([vetices_robot], cyan)
        draw_poligonos([vetices_goal], red)
        print("\nDibujamos X_goal y X_start")

        for obstaculo in self.obstaculos:
            Poligono = Polygon(obstaculo, color=orange) #Para mostrar
            view.add(Poligono)
        print("\nDibujamos los obstaculos")

        #Mostramos los vertices
        for vertex in self.vertices:
            centro = [vertex[0],vertex[1]]
            dot = Rectangle(center = centro, width=5, height=5, color=blue)
            view.add(dot)
        print("\nDibujamos los vertices")

       
        if flag:#Si se encontro goal
            print("\nSE ENCONTRO X_goal")
            #El calculo del camino se realiza en RRT_on_DDR_move.py
            """Estados = look_for_padre(self.padres,self.vertices,self.goal,len(self.vertices)-1)
            print("\nSe calculo la ruta")
            for estado in Estados:
              block = Polygon(create_estado(estado[0],estado[1],estado[2]),color=white,parent = view.scene)
              view.add(block)
            print("\nDibujamos la ruta")"""
            

        else:#Si unicamente dibujaremos los estados iniciales  fianales
            print("\nNO SE ENCONTRO X_goal")

#Nota: se omitieron los dibujos de las aritstas y el camino de x_goal hacia x_start debido a errores en la visualización
#########################################################################################################

#Declaracion de obstaculos
vert1 = create_rect(np.array([80,100]),40,40)
vert2 = create_rect(np.array([200,100]),40,40)
vert3 = create_rect(np.array([360,100]),40,40)
vert4 = create_rect(np.array([450,100]),40,40)
vert5 = create_rect(np.array([600,100]),40,40)
vert6 = create_rect(np.array([50,280]),40,40)
vert7 = create_rect(np.array([170,280]),40,40)
vert8 = create_rect(np.array([300,280]),40,40)
vert9 = create_rect(np.array([500,280]),40,40)
vert10 = create_rect(np.array([600,280]),40,40)
vert11 = create_rect(np.array([150,480]),40,40)
vert12 = create_rect(np.array([350,480]),40,40)
vert13 = create_rect(np.array([500,480]),40,40)
vert14 = create_rect(np.array([100,630]),40,40)
vert15 = create_rect(np.array([270,630]),40,40)
vert16 = create_rect(np.array([500,630]),40,40)

obstaculos = [vert1,vert2,vert3,vert4,vert5,vert6,vert7,vert8,vert9,vert10,vert11,vert12,vert13,vert14,vert15,vert16]

#Valores para RRT
scene_width = 700
scene_height = 700
r = 6
b = 25
X_start = [40,40,np.pi/2]
#X_start = [scene_width/2,scene_height/2,np.pi/2]
X_goal = [600,640,0]
x_lim = [0,scene_width]
y_lim = [0,scene_height]
Dt = 1.0
h = 0.1
max_iteraciones = 5000#Intentaremos esta cantidad, su doble, triple y cuadruple

RRT_DDR = RRT(X_start, b, r, Dt, h, X_goal, obstaculos,x_lim, y_lim)

found_path = RRT_DDR.plan(max_iteraciones*4)

print(f"\nTerminamos RRT con {len(RRT_DDR.vertices)} vertices y {len(RRT_DDR.edges)} aristas")

canvas = SceneCanvas(keys='interactive', title='RRT_on_DDR_routes',
                     show=True, size = (scene_width, scene_height), autoswap=False, vsync=True)
view = canvas.central_widget.add_view()
view.bgcolor = gray
print("\nCreamos la ventana")


RRT_DDR.visualize(found_path)
print("\nVisualizamos RRT")

timer = app.Timer()
#timer.connect(update)
timer.start()

if __name__ == '__main__':
    if sys.flags.interactive != 1:
        app.run()
