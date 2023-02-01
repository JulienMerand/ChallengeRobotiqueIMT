"""
Programme principal pour le challenge Robotique 2023 - IMT Nord Europe

Julien MÉRAND - Quentin DESMARTIN
"""

###########################################################################################################################################################
# IMPORT
###########################################################################################################################################################

import sys
import time
import math
import cv2
import numpy as np
import igraph as ig
import matplotlib.pyplot as plt
try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

###########################################################################################################################################################
# VARIABLES
###########################################################################################################################################################

DEFAULT_IP = '127.0.0.1'
VREP_PORT = 19997
UPDATE_PERIOD = 100
siID = -1

MON_PI = 3.141592654

MOBILE_BASE = 'Robot'
LEFT_MOTOR = 'LeftMotor'
RIGHT_MOTOR = 'RightMotor'
CAMERA = 'MainCamera'
CORRECTION_COEFF = (1.25 * 36.0 / 30.0)       # ->facteur correctif pour assurer des deplacements en B.O a peu pres corrects...
ENTRAXE = 0.2
WHEEL_RADIUS = 0.1                                             

Vmax0 = 1           # Vitesse max a vide
coef_a = 1/600      # Pondaration Masse-Vitesse
coef_ac = 7.41e-5   # Pondaration Masse-Carburant
coef_bc = 1/15      # Consommation a vide

###########################################################################################################################################################
# FONCTIONS
###########################################################################################################################################################

def InitAllHandles():
    global iBaseHandle, iLeftMotor, iRightMotor #, iCamera
    #..........................................
    # recuperation du handle sur la base mobile 
    #..........................................
    siErrorCode, iBaseHandle = vrep.simxGetObjectHandle(siID, MOBILE_BASE, vrep.simx_opmode_blocking)
    if( siErrorCode != vrep.simx_error_noerror ):
        print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
        print('         code de retour V-REP = ' + str(siErrorCode))
        vrep.simxFinish(siID)
        return(-1)
    print("handle sur la base mobile : OK = " + str(iBaseHandle) )
    #............................................
    # recuperation du handle sur le moteur gauche 
    #............................................
    siErrorCode, iLeftMotor = vrep.simxGetObjectHandle(siID, LEFT_MOTOR, vrep.simx_opmode_blocking)
    if( siErrorCode != vrep.simx_error_noerror ):
        print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
        print('         code de retour V-REP = ' + str(siErrorCode))
        vrep.simxFinish(siID)
        return(-2)
    print("handle sur le moteur gauche : OK = " + str(iLeftMotor) )
    #............................................
    # recuperation du handle sur le moteur droit 
    #............................................
    siErrorCode, iRightMotor = vrep.simxGetObjectHandle(siID, RIGHT_MOTOR, vrep.simx_opmode_blocking)
    if( siErrorCode != vrep.simx_error_noerror ):
        print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
        print('         code de retour V-REP = ' + str(siErrorCode))
        vrep.simxFinish(siID)
        return(-3)
    print("handle sur le moteur droit : OK = " + str(iRightMotor) )
    #............................................
    # recuperation du handle sur la camera 
    #............................................
    # siErrorCode, iCamera = vrep.simxGetObjectHandle(siID, CAMERA, vrep.simx_opmode_blocking)
    # if( siErrorCode != vrep.simx_error_noerror ):
    #     print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
    #     print('         code de retour V-REP = ' + str(siErrorCode))
    #     vrep.simxFinish(siID)
    #     return(-4)
    # print("handle sur le moteur droit : OK = " + str(iCamera) )
    return(0)

def Set_Immobile():
    '''
    Immobilisation du Robot au départ de la simu. 
    '''
    siError = vrep.simxSetJointTargetVelocity(siID, iLeftMotor, 0.0, vrep.simx_opmode_blocking)
    if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
        print('Turn() : ERREUR ---> appel a simxSetJointTargetVelocity() [gauche]')
        print('code d erreur V-REP = ' + str(siError))
        return(-1)
    siError = vrep.simxSetJointTargetVelocity(siID, iRightMotor, 0.0, vrep.simx_opmode_blocking)
    if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
        print('Turn() : ERREUR ---> appel a simxSetJointTargetVelocity() [droite]')
        print('code d erreur V-REP = ' +  str(siError))
        return(-2)
    print("Robot à l'arrêt !")
    return(0)

def GetMobileBasePosition():  
    #............................
    # recuperation de la position
    #............................
    siError, Pos =  vrep.simxGetObjectPosition(siID, iBaseHandle, -1 ,vrep.simx_opmode_blocking)
    if (siError != vrep.simx_return_ok ):
        print('GetMobileBasePosition() : ERREUR ---> appel a simxGetObjectPosition().\n')
        print('code d erreur COPPELIA = ', str(siError) )
        return -2,[],[]
  #.............................
  # recuperation de l'orientation
  #..............................
    siError, Ori =  vrep.simxGetObjectOrientation(siID, iBaseHandle, -1 ,vrep.simx_opmode_blocking)
    if (siError != vrep.simx_return_ok ):
        print('GetMobilePosition() : ERREUR ---> appel a simxGetObjectOrientation().\n')
        print('code d erreur COPPELIA = ', str(siError) )
        return -1,Pos,[]
    Ori[2] += MON_PI
    # OK
    return(0,Pos,Ori[2])

# def InitVisionSystem():
#     siError, imgResolution, imgData = vrep.simxGetVisionSensorImage(siID, iCamera, 0, vrep.simx_opmode_streaming )
#     if(siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag):
#         print('InitVisionSystem() : ERREUR -->appel a simxGetVisionSensorImage()')
#         print('                     code d erreur COPPELIA = ' + str(siError))
#         return(-1)
#     print("handle MainCamera = " + str( iCamera))
#     return( 0 )

def COP2CV2ImageWrapper( imgRes, imgData ):
    imgShape = [imgRes[1],imgRes[0],3]
    cvImg = np.zeros(imgShape, dtype = np.uint8 )
    index = 0
    for i in range(imgRes[1]):
        for j in range(imgRes[0]):
            # COPPELIA travaille en RGB, et OpenCV en BGR...
            # il faut de plus inverser l'ordre des lignes
            cvImg[imgRes[1] -1 - i,j,2] = imgData[index]
            cvImg[imgRes[1] -1 - i,j,1] = imgData[index+1]
            cvImg[imgRes[1] -1 - i,j,0] = imgData[index+2]
            index = index + 3
    return cvImg

def GrabImageFromCam():
    siError, imgResolution, imgData = vrep.simxGetVisionSensorImage(siID, iCamera, 0, vrep.simx_opmode_streaming )
    if( siError != vrep.simx_return_ok ) and (siError != vrep.simx_return_novalue_flag):
        print('GrabImageFromCam() : ERREUR --> appel a simxGetVisionSensorImage() 1')
        print('                     code d erreur COPPELIA = ' + str(siError))
        return [], -1
    time.sleep(0.1)
    siError, imgResolution, imgData = vrep.simxGetVisionSensorImage(siID, iCamera, 0, vrep.simx_opmode_buffer )
    if( siError != vrep.simx_return_ok ) and (siError != vrep.simx_return_novalue_flag):
        print('GrabImageFromCam() : ERREUR --> appel a simxGetVisionSensorImage() 2')
        print('                     code d erreur COPPELIA = ' + str(siError))
        return [], -1
    # Appel du "Wrapper" pour convertir au format
    # RGB OpenCV : 
    cvImg = COP2CV2ImageWrapper(imgResolution, imgData)
    # fini
    return cvImg, 0


def Map_vers_Cylindres(PosInit) :
    ''' Retourne la liste des cylindres avec le bon format et avec pour cylindre numero 0 la position initiale du Robo'''
    DataMap = np.loadtxt(r'C:\Users\julie\Documents\Ecole\IMT Nord Europe\M1\Challenge Robotique\Challenge\donnees-map.txt', dtype=float)
    #affichage des donnees de la carte
    x=DataMap[:,0]
    y=DataMap[:,1]
    t=DataMap[:,2]
    Liste_Cylindres = [[0,PosInit,0,10000000]]  #Pt_i = [i,[Xi, Yi], Mi, Ri]
    for i in range(len(DataMap)):
        Pt_i = [i+1,[DataMap[i][0],DataMap[i][1]]]
        if DataMap[i][2] == 1:
            Pt_i.append(10)
            Pt_i.append(250)
            Pt_i.append(1)
        elif DataMap[i][2] == 2:
            Pt_i.append(30)
            Pt_i.append(600)
            Pt_i.append(2)
        elif DataMap[i][2] == 3:
            Pt_i.append(50)
            Pt_i.append(1500)
            Pt_i.append(3)
        else :
            return ("Probleme dans DataMap...")
        Liste_Cylindres.append(Pt_i)
    return(Liste_Cylindres)



def Plot_Cylindres(c, edges):
    ''' Plot les cylindres à leurs coordonnées '''
    X = [cyl[1][0] for cyl in c]
    Y = [cyl[1][1] for cyl in c]
    N = [str(cyl[0]) for cyl in c]
    plt.scatter(X,Y)
    for i, txt in enumerate(N):
        plt.annotate(txt, (X[i], Y[i]), weight='bold', size=16)
    for (p1,p2) in edges:
        point1 = c[p1][1]
        point2 = c[p2][1]
        x_values = [point1[0], point2[0]]
        y_values = [point1[1], point2[1]]
        plt.plot(x_values, y_values, 'bo', linestyle="-")
    # plt.show()

def distancePP(A,B):
    return math.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)

def Poids_i_vers_j(Pt_i, Pt_j, ponderationTps=1.0, ponderationCarbu=1.0):
    ''' Retourne le poids du graphe vu du dessus entre le cylindre i et le cylindre j'''
    'Pt_i = [i,[Xi, Yi], Mi, Ri]'
    Mi = Pt_i[2]
    Mj = Pt_j[2]
    Rj = Pt_j[3]
    # dist = math.sqrt((Pt_i[1][0]-Pt_j[1][0])**2 + (Pt_i[1][1]-Pt_j[1][1])**2)
    dist = distancePP(Pt_i[1],Pt_j[1])
    Cout_i_vers_j = ponderationTps * dist / Vmax0*(1-math.exp(-coef_a*Mi)) + ponderationCarbu * (coef_ac * Mi + coef_bc) * dist
    Gain_i_vers_j = Rj
    return round((Cout_i_vers_j / Gain_i_vers_j),2)

def droite(A,B):
    '''
    Renvoie les coefficients de la droite passant par A et B
    (m,p)  --->  y = m*x + p 
    '''
    xA, yA, xB, yB = A[0], A[1], B[0], B[1]
    m = ((yB-yA)/(xB-xA))
    p = yB - m*xB
    return (m,p)

def distancePD(Pnt,Drte):
    '''Renvoie la distance du point à la droite'''
    xP, yP = Pnt[0], Pnt[1]
    (m,p) = Drte
    return abs((-m*xP+yP-p)/(math.sqrt((-m)**2+1)))

def ConstructGraph(cylindres, R=0.5):
    '''
    Construit le graph correspondant aux différents cylindres \n
    nbre_cy = int | cylindres = [cyl1, cyl2, ...] \n
    cyl5 = [5, [x,y], M, R]
    '''
    nbre_cyl = len(cylindres)
    g = ig.Graph(n=nbre_cyl, directed=True)
    g.vs["num"] = [str(cyl[0]) for cyl in cylindres]
    g.vs["masse"] = [str(cyl[2]) for cyl in cylindres]
    g.vs["gain"] = [str(cyl[3]) for cyl in cylindres]
    poids = [] 
    for cyl1 in cylindres:
        # print(str(cyl1))
        for cyl2 in cylindres:
            if cyl1 != cyl2:
                # print("\t-------- " + str(cyl2))
                b = True
                i = 0
                while b and i < nbre_cyl:
                    cyl3 = cylindres[i]
                    test_X = (cyl1[1][0]<cyl2[1][0] and cyl1[1][0]<cyl3[1][0] and cyl3[1][0]<cyl2[1][0]) or (cyl1[1][0]>cyl2[1][0] and cyl1[1][0]>cyl3[1][0] and cyl3[1][0]>cyl2[1][0])
                    test_Y = (cyl1[1][1]<cyl2[1][1] and cyl1[1][1]<cyl3[1][1] and cyl3[1][1]<cyl2[1][1]) or (cyl1[1][1]>cyl2[1][1] and cyl1[1][1]>cyl3[1][1] and cyl3[1][1]>cyl2[1][1])
                    test_dist = distancePP(cyl1[1],cyl2[1]) > distancePP(cyl1[1],cyl3[1])
                    # print("\t\t-------- " + str(cyl3) + " | " + str(test_X) + " " + str(test_Y) + " " + str(test_dist))
                    if cyl3 != cyl1 and cyl3 != cyl2 and (test_X or test_Y) and test_dist:
                        dte = droite(cyl1[1],cyl2[1])
                        distance = distancePD(cyl3[1], dte)
                        # print("\t\t\t\tdroite : " + str(dte) + " | distance : " + str(distance))
                        h = distancePP(cyl1[1],cyl2[1])/4
                        if distance < h:
                            b = False
                            # print("\t\t\t\tedge NOK")
                    i += 1
                if b and i==nbre_cyl and ((cyl1[0],cyl2[0]) not in list(g.get_edgelist())) :
                    g.add_edges([(cyl1[0],cyl2[0]),(cyl2[0],cyl1[0])])
                    poids.append(Poids_i_vers_j(cyl1,cyl2))
                    poids.append(Poids_i_vers_j(cyl2,cyl1))
                    # print("\t\t\t\tedge OK")
                    # print("\t\t\t\tedge : " + str(g.get_edgelist()))
                    # print("\t\t\t\tpoids : " + str(poids))
    g.es["weight"] = poids
    A = list(g.get_adjacency(attribute='weight', default=1e10))
    return g, A

def Afficher_Graphe(grph):
    label = []
    for i in range(int(grph.vcount())):
        chaine = grph.vs[i]["num"] #+ " , " + grph.vs[i]["masse"] + " , " + grph.vs[i]["gain"]
        label.append(chaine)
    visual_style = { 
                     "edge_width": 1.0,
                    #  "vertex_size": 0.1,
                     "edge_arrow_size" : 0.005,
                    #  "edge_arrow_width" : 0.5,
                     "vertex_label" : label,
                    #  "edge_label" : grph.es["poids"],
                    #  "palette": "heat",
                    #  "layout": "fruchterman_reingold"
                     "layout" : "kk" }
    # layout = g.layout("drl")
    fig, ax = plt.subplots()
    ig.plot(grph, target=ax, autocurved=False, **visual_style)
    fig.tight_layout()
    plt.show()

def shortest_path(adj_matrix):
    ''' Retourne le chemin le plus court parcourant tout les noeuds du graphe '''
    # Helper function to calculate path length
    def path_len(path):
        return sum(adj_matrix[i][j] for i, j in zip(path, path[1:]))

    # Set of all nodes to visit
    to_visit = set(range(len(adj_matrix)))

    # Current state {(node, visited_nodes): shortest_path}
    state = {(i, frozenset([0, i])): [0, i] for i in range(1, len(adj_matrix[0]))}

    for k in range(len(adj_matrix) - 2):
        next_state = {}
        # print(len(state))
        print("|"+k*"#"+(len(adj_matrix)-2-k-1)*"-"+"|")
        for position, path in state.items():
            current_node, visited = position
            # Check all nodes that haven't been visited so far
            for node in to_visit - visited:
                new_path = path + [node]
                new_pos = (node, frozenset(new_path))
                # Update if (current node, visited) is not in next state or we found shorter path
                if new_pos not in next_state or path_len(new_path) < path_len(next_state[new_pos]):
                    next_state[new_pos] = new_path

        state = next_state

    # Find the shortest path from possible candidates
    shortest = min((path + [0] for path in state.values()), key=path_len)
    print('path: {0}, length: {1}'.format(shortest, path_len(shortest)))
    return (shortest, path_len(shortest))

def angle_turn(Pos_Cylindre, Pos_Robo, Orientation_Robo):
    ''' Retourne la valeur de l'angle entre le Robo et le cylindre vise'''
    # Pos_Cylindre = [Xc, Yc]
    # Pos_Robo = [Xr, Yr]
    # Orientation_Robo = angle autour de Z entre - pi et pi
    vecteur_ecart = [Pos_Cylindre[0]-Pos_Robo[0],Pos_Cylindre[1]-Pos_Robo[1]]
    vecteur_orientation = [math.cos(Orientation_Robo), math.sin(Orientation_Robo)]
    prod_scalaire = (float)(vecteur_ecart[0]*vecteur_orientation[0] + vecteur_ecart[1] * vecteur_orientation[1])
    norme_ecart = math.sqrt(vecteur_ecart[0]**2 + vecteur_ecart[1]**2)
    return(math.acos(prod_scalaire/norme_ecart))

def Go(speed):
    '''
    Immobilisation du Robot au départ de la simu. 
    '''
    siError = vrep.simxSetJointTargetVelocity(siID, iLeftMotor, speed, vrep.simx_opmode_blocking)
    if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
        print('Turn() : ERREUR ---> appel a simxSetJointTargetVelocity() [gauche]')
        print('code d erreur V-REP = ' + str(siError))
        return(-1)
    siError = vrep.simxSetJointTargetVelocity(siID, iRightMotor, speed, vrep.simx_opmode_blocking)
    if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
        print('Turn() : ERREUR ---> appel a simxSetJointTargetVelocity() [droite]')
        print('code d erreur V-REP = ' +  str(siError))
        return(-2)
    print("Robot en marche !")
    return(0)

def DEG2RAD(x):
  y = MON_PI * ( x / 180.0)
  return( y )

def RAD2DEG(x):
  y = 180.0 * ( x / MON_PI )
  return( y )

def Go_To_Cylindre(Tab_Cyl ,num):
    seuil = DEG2RAD(15)
    seuil_centre = DEG2RAD(5)
    Pos_Cyl = Tab_Cyl[num][1]
    ret, Pos_Rob, Ori_Rob = GetMobileBasePosition()
    if ret >= 0:
        while distancePP(Pos_Rob, Pos_Cyl) > 1.05 : # robot pas dans cylindre + delta
            ret, Pos_Rob, Ori_Rob = GetMobileBasePosition()
            print("Cyl : ",Pos_Cyl)
            if ret >= 0:
                ang = angle_turn(Pos_Cyl, Pos_Rob, Ori_Rob)
                if ang > seuil:
                    while ang > seuil_centre:
                        time.sleep(0.2)
                        print("Left")
                        print(RAD2DEG(ang))
                        siError = vrep.simxSetJointTargetVelocity(siID, iRightMotor, 3.0, vrep.simx_opmode_blocking)
                        if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
                            print('main() : ERREUR ---> appel a simxSetJointTargetVelocity() [gauche]')
                            print('code d erreur COPPELIA = ' + str(siError))
                            vrep.simxFinish(siID)
                            exit()
                        siError = vrep.simxSetJointTargetVelocity(siID, iLeftMotor, 0.0, vrep.simx_opmode_blocking)
                        if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
                            print('main() : ERREUR ---> appel a simxSetJointTargetVelocity() [gauche]')
                            print('code d erreur COPPELIA = ' + str(siError))
                            vrep.simxFinish(siID)
                            exit()
                        ret, Pos_Rob, Ori_Rob = GetMobileBasePosition()
                        if ret >= 0:
                            ang = angle_turn(Pos_Cyl, Pos_Rob, Ori_Rob)
                elif ang < -seuil:
                    while ang < -seuil_centre:
                        time.sleep(0.2)
                        print("Right")
                        siError = vrep.simxSetJointTargetVelocity(siID, iLeftMotor, 3.0, vrep.simx_opmode_blocking)
                        if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
                            print('main() : ERREUR ---> appel a simxSetJointTargetVelocity() [gauche]')
                            print('code d erreur COPPELIA = ' + str(siError))
                            vrep.simxFinish(siID)
                            exit()
                        siError = vrep.simxSetJointTargetVelocity(siID, iRightMotor, 0.0, vrep.simx_opmode_blocking)
                        if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
                            print('main() : ERREUR ---> appel a simxSetJointTargetVelocity() [gauche]')
                            print('code d erreur COPPELIA = ' + str(siError))
                            vrep.simxFinish(siID)
                            exit()
                        ret, Pos_Rob, Ori_Rob = GetMobileBasePosition()
                        if ret >= 0:
                            ang = angle_turn(Pos_Cyl, Pos_Rob, Ori_Rob)
                else:
                    # Tout droit
                    time.sleep(0.2)
                    if Go(5.0) < 0:
                        print("ERREUR : main() ---> appel a Go() : Erreur pour un des moteurs")
                        vrep.simxFinish(siID)
                        exit()
        if Set_Immobile() < 0:
            print("6")
            print("ERREUR : main() ---> appel a Set_Immobile() : Erreur pour un des moteurs")
            vrep.simxFinish(siID)
            exit()


###########################################################################################################################################################
# INIT
###########################################################################################################################################################

#.........................................
# tentative de connexion au serveur V-REP 
#.........................................
argc = len(sys.argv)
if( argc == 1 ):
    szServerAddr = DEFAULT_IP
else:
    szServerAddr = sys.argv[1]
vrep.simxFinish(-1)
time.sleep(0.1)
siID = vrep.simxStart(szServerAddr, VREP_PORT, 1, 1, 10000, 50)
if( siID < 0):
  print('ERREUR : main() ---> appel a simxStart() : impossible de se connecter a ' +  sys.argv[1])
  print('         valeur de retour = ' + str(siID))
  exit()
print("Connexion au serveur COPPELIA fonctionnant sur " + szServerAddr + "etablie...")

#.........................................
# Initialisation des Handles
#.........................................
if InitAllHandles() < 0:
    print("ERREUR : main() ---> appel a InitAllHandles() : un des handles est indisponible")
    vrep.simxFinish(siID)
    exit()

if Set_Immobile() < 0:
    print("ERREUR : main() ---> appel a Set_Immobile() : Erreur pour un des moteurs")
    vrep.simxFinish(siID)
    exit()

time.sleep(0.1)

#.........................................
# Initialisation de la camera
#.........................................
# print("Initialisation du systeme de vision...")
# if InitVisionSystem() < 0:
#     print("ERREUR : main() ---> appel a InitVisionSystem() : echec de l'initialisation...")
#     vrep.simxFinish(siID)
#     exit()
# print("OK")
# time.sleep(0.1) 


#.........................................
# Récuperation des cylindres 
#.........................................



###########################################################################################################################################################
# PROGRAMME
###########################################################################################################################################################

# .........................................
# Creation du graphe 
# .........................................

ret, PosInit, OriInit = GetMobileBasePosition()

Tab_Cylindres = Map_vers_Cylindres(PosInit)
# c = [[0, [7.7, 8.7], 98, 15.68], [1, [2.8, 5.4], 97, 13.58], [2, [5.3, 6.5], 63, 8.82], [3, [3.6, 1.1], 46, 5.52], [4, [5.4, 6.9], 75, 12]]
# c = del_cyl(c)
# print(c)

# graph, adj_matrix = ConstructGraph(c[:15])
# Plot_Cylindres(c, graph.get_edgelist())
# # print(adj_matrix)
# path, length = shortest_path(adj_matrix)
# print("Affichage en cours")
# Afficher_Graphe(graph)

print("je pars")
Go_To_Cylindre(Tab_Cylindres,9)

# #.........................................
# # Recuperation du frame de la camera 
# #.........................................
# for i in range(50):
#     while True:
#         cvImg, ret = GrabImageFromCam()
#         time.sleep(0.1)
#         if ret >= 0:
#             break
#     # affichage
#     cv2.imshow('CAMERA',cvImg)
#     cv2.waitKey(2)





###########################################################################################################################################################
# DECONNECTION DU SERVEUR
###########################################################################################################################################################

time.sleep(1)
print("deconnexion du serveur COPPELIA...")
vrep.simxFinish(siID)
print("OK")

