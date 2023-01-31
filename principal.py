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


VREP_PORT = 19997
UPDATE_PERIOD = 100

MON_PI = 3.141592654

MOBILE_BASE = 'RoBo'
LEFT_MOTOR = 'leftMotor'
RIGHT_MOTOR = 'rightMotor'
CORRECTION_COEFF = (1.25 * 36.0 / 30.0)       # ->facteur correctif pour assurer des deplacements en B.O a peu pres corrects...
ENTRAXE = 0.2
WHEEL_RADIUS = 0.1                                             

Vmax0 = 20      # Vitesse max a vide
coef_a = 10     # Pondaration Masse-Vitesse
coef_ac = 10    # Pondaration Masse-Carburant
coef_bc = 10    # Consommation a vide

###########################################################################################################################################################
# FONCTIONS
###########################################################################################################################################################

def DEG2RAD(x):
    ''' deg --> rad '''
    y = MON_PI * ( x / 180.0)
    return( y )


def Turn( dbAngle, dbVel):
    '''
    Ang = deg | Vel = deg/s 
    Left : ANG > 0.0
    Right : ANG < 0.0
    '''
    #......
    # init 
    #......
    dbVelRad = DEG2RAD(math.fabs(dbVel))
    w = (0.5 * dbVelRad * ENTRAXE)/WHEEL_RADIUS
    dT = DEG2RAD(math.fabs(dbAngle)) / dbVelRad
    iSleepTime = dT
    if (dbAngle >= 0.0):
        wR = w
        wL = -w
    else:
        wR = -w
        wL = w
    #....................................*/
    # Application des vitesses au moteur */
    #....................................*/
    siError = vrep.simxSetJointTargetVelocity(siID, iLeftMotor, wL, vrep.simx_opmode_blocking)
    if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
        print('Turn() : ERREUR ---> appel a simxSetJointTargetVelocity() [gauche]')
        print('code d erreur V-REP = ' + str(siError))
        return(-1)

    siError = vrep.simxSetJointTargetVelocity(siID, iRightMotor, wR, vrep.simx_opmode_blocking)
    if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
        print('Turn() : ERREUR ---> appel a simxSetJointTargetVelocity() [droite]')
        print('code d erreur V-REP = ' +  str(siError))
        return(-1)
    #................................................................
    # attente de la realisation du mouvement : on peut mieux faire ! 
    #................................................................
    iTotTime = 0;
    while (iTotTime < iSleepTime ):
        time.sleep(0.001 * UPDATE_PERIOD * CORRECTION_COEFF )
        iTotTime = iTotTime + 0.001 * UPDATE_PERIOD
    #...........................
    # on pense a s'arreter... : 
    #...........................
    wL = 0.0
    wR = 0.0
    siError = vrep.simxSetJointTargetVelocity(siID, iLeftMotor, wL, vrep.simx_opmode_blocking)
    if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
        print('Turn() : ERREUR ---> appel a simxSetJointTargetVelocity() [gauche]')
        print('code d erreur V-REP = ' + str(siError))
        return(-1)

    siError = vrep.simxSetJointTargetVelocity(siID, iRightMotor, wR, vrep.simx_opmode_blocking)
    if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
        print('Turn() : ERREUR ---> appel a simxSetJointTargetVelocity() [droite]')
        print('code d erreur V-REP = ' +  str(siError))
        return(-1)
    # OK
    return(0)


def Go(dbDist, dbVel):
    ''' 
    Dist = m | Vel = m/s
    Marche arrière : Dist < 0
    '''
    #......
    # init 
    #......
    w = math.fabs(dbVel) / WHEEL_RADIUS
    dT = math.fabs(dbDist / dbVel)
    iSleepTime = dT
    if (dbDist < 0.0):
        w = -w       
    #....................................*/
    # Application des vitesses au moteur */
    #....................................*/
    siError = vrep.simxSetJointTargetVelocity(siID, iLeftMotor, w, vrep.simx_opmode_blocking)
    if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
        print('Go() : ERREUR ---> appel a simxSetJointTargetVelocity() [gauche]')
        print('code d erreur V-REP = ' + str(siError))
        return(-1)

    siError = vrep.simxSetJointTargetVelocity(siID, iRightMotor, w, vrep.simx_opmode_blocking)
    if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
        print('Go() : ERREUR ---> appel a simxSetJointTargetVelocity() [droite]')
        print('code d erreur V-REP = ' +  str(siError))
        return(-1)
    #................................................................
    # attente de la realisation du mouvement : on peut mieux faire ! 
    #................................................................
    iTotTime = 0;
    while (iTotTime < iSleepTime ):
        time.sleep(0.001 * UPDATE_PERIOD * CORRECTION_COEFF )
        iTotTime = iTotTime + 0.001 * UPDATE_PERIOD
    #...........................
    # on pense a s'arreter... : 
    #...........................
    w = 0.0
    siError = vrep.simxSetJointTargetVelocity(siID, iLeftMotor, w, vrep.simx_opmode_blocking)
    if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
        print('Go() : ERREUR ---> appel a simxSetJointTargetVelocity() [gauche]')
        print('code d erreur V-REP = ' + str(siError))
        return(-1)

    siError = vrep.simxSetJointTargetVelocity(siID, iRightMotor, w, vrep.simx_opmode_blocking)
    if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
        print('Go() : ERREUR ---> appel a simxSetJointTargetVelocity() [droite]')
        print('code d erreur V-REP = ' +  str(siError))
        return(-1)
    # OK
    return(0)

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
    g.vs["num"] = [str(k) for k in range(len(cylindres))]
    g.vs["masse"] = [str(cyl[2]) for cyl in cylindres]
    g.vs["gain"] = [str(cyl[3]) for cyl in cylindres]
    poids = [] 

    for cyl1 in cylindres:
        print(str(cyl1))
        for cyl2 in cylindres:
            if cyl1 != cyl2:
                print("\t-------- " + str(cyl2))
                b = True
                i = 0
                while b and i < nbre_cyl:
                    cyl3 = cylindres[i]
                    test_X = max(cyl1[1][0],cyl2[1][0]) - cyl3[1][0] > 0 and cyl3[1][0] - min(cyl1[1][0],cyl2[1][0]) > 0
                    test_Y = max(cyl1[1][1],cyl2[1][1]) - cyl3[1][1] > 0 and cyl3[1][1] - min(cyl1[1][0],cyl2[1][1]) > 0
                    if cyl3 != cyl1 and cyl3 != cyl2 and (test_X or test_Y) :
                        print("\t\t-------- " + str(cyl3))
                        dte = droite(cyl1[1],cyl2[1])
                        distance = distancePD(cyl3[1], dte)
                        print("\t\t\t\tdroite : " + str(dte) + " | distance : " + str(distance))
                        if distance < distancePP(cyl1[1],cyl2[1])/6:
                            b = False
                            print("\t\t\t\tedge NOK")
                    i += 1
                if b and i==nbre_cyl:
                    g.add_edges([(cyl1[0],cyl2[0])])
                    poids.append(Poids_i_vers_j(cyl1,cyl2,))
                    print("\t\t\t\tedge OK")
                    print("\t\t\t\tedge : " + str(g.get_edgelist()))
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
    # Helper function to calculate path length
    def path_len(path):
        return sum(adj_matrix[i][j] for i, j in zip(path, path[1:]))

    # Set of all nodes to visit
    to_visit = set(range(len(adj_matrix)))

    # Current state {(node, visited_nodes): shortest_path}
    state = {(i, frozenset([0, i])): [0, i] for i in range(1, len(adj_matrix[0]))}
    # print(state.items())
    # print(len(adj_matrix))
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




import random
def Test_Cylindres(n=10):
    cylindres = []
    for i in range(n):
        x, y = round(random.randint(0,100)/10,1), round(random.randint(0,100)/10,1)
        M = random.randint(25,100)
        R = round(M*0.2*random.randint(5,10)*0.1,2)
        cyl = [i, [x,y], M, R]
        cylindres.append(cyl)
    return cylindres

c = Test_Cylindres()
# c = [[0, [7.7, 8.7], 98, 15.68], [1, [2.8, 5.4], 97, 13.58], [2, [5.3, 6.5], 63, 8.82], [3, [3.6, 1.1], 46, 5.52], [4, [5.4, 6.9], 97, 13.58]]
graph, adj_matrix = ConstructGraph(c)
Plot_Cylindres(c, graph.get_edgelist())
# print(adj_matrix)
# path, length = shortest_path(adj_matrix)

print("Affichage en cours")
Afficher_Graphe(graph)










'''
###########################################################################################################################################################
# INIT
###########################################################################################################################################################

#.........................................
# tentative de connexion au serveur V-REP 
#.........................................
vrep.simxFinish(-1)
time.sleep(1)

siID = vrep.simxStart('127.0.0.1', VREP_PORT, 1, 1, 10000, 50)

if( siID < 0):
  print('ERREUR : main() ---> appel a simxStart() : impossible de se connecter a ' +  sys.argv[1])
  print('         valeur de retour = ' + str(siID))
  exit()

#..........................................
# recuperation du handle sur la base mobile 
#..........................................
siErrorCode, iBaseHandle = vrep.simxGetObjectHandle(siID, MOBILE_BASE, vrep.simx_opmode_blocking)
if( siErrorCode != vrep.simx_error_noerror ):
  print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
  print('         code de retour V-REP = ' + str(siErrorCode))
  vrep.simxFinish(siID)
  exit()

#............................................
# recuperation du handle sur le moteur gauche 
#............................................
siErrorCode, iLeftMotor = vrep.simxGetObjectHandle(siID, LEFT_MOTOR, vrep.simx_opmode_blocking)
if( siErrorCode != vrep.simx_error_noerror ):
  print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
  print('         code de retour V-REP = ' + str(siErrorCode))
  vrep.simxFinish(siID)
  exit()

#............................................
# recuperation du handle sur le moteur droit 
#............................................
siErrorCode, iRightMotor = vrep.simxGetObjectHandle(siID, RIGHT_MOTOR, vrep.simx_opmode_blocking)
if( siErrorCode != vrep.simx_error_noerror ):
  print('ERREUR : main() ---> apppel a simxGetObjectHandle()\n')
  print('         code de retour V-REP = ' + str(siErrorCode))
  vrep.simxFinish(siID)
  exit()

#.........................................  
# immobilisation du Robot  : 
#.........................................
siError = vrep.simxSetJointTargetVelocity(siID, iLeftMotor, 0.0, vrep.simx_opmode_blocking)
if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
  print('Turn() : ERREUR ---> appel a simxSetJointTargetVelocity() [gauche]')
  print('code d erreur V-REP = ' + str(siError))
  exit()
siError = vrep.simxSetJointTargetVelocity(siID, iRightMotor, 0.0, vrep.simx_opmode_blocking)
if ((siError != vrep.simx_return_ok) and (siError != vrep.simx_return_novalue_flag)):
  print('Turn() : ERREUR ---> appel a simxSetJointTargetVelocity() [droite]')
  print('code d erreur V-REP = ' +  str(siError))
  exit()

print('Connecté à la simulation !')

###########################################################################################################################################################
# NAVIGATION
###########################################################################################################################################################





###########################################################################################################################################################
# DECONNECTION DU SERVEUR
###########################################################################################################################################################

time.sleep(1)
vrep.simxFinish(siID)

'''