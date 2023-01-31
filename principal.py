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


def droite(A,B):
    '''
    Renvoie les coefficients de la droite passant par A et B
    (m,p)  --->  y = m*x + p 
    '''
    xA, xB, yA, yB = A[0], A[1], B[0], B[1]
    m = ((yB-yA)/(xB-xA))
    p = yB - m*xB
    return (m,p)

def distancePD(Pnt,Drte):
    '''Renvoie la distance du point à la droite'''
    xP, yP = Pnt[0], Pnt[1]
    (m,p) = Drte
    return abs((-m*xP+yP-p)/(math.sqrt((-m)**2+1)))

def ConstructGraph(nbre_cyl, cylindres, R=0.5):
    '''
    Construit le graph correspondant aux différents cylindres \n
    nbre_cy = int | cylindres = [cyl1, cyl2, ...] \n
    cyl5 = [5, [x,y], M, R]
    '''
    g = ig.Graph(n=nbre_cyl)

    g.vs["num"] = [str(k) for k in range(len(cylindres))]
    g.vs["masse"] = [str(cyl[2]) for cyl in cylindres]
    g.vs["gain"] = [str(cyl[3]) for cyl in cylindres]    

    for cyl1 in cylindres:
        for cyl2 in cylindres:
            if cyl1 != cyl2:
                b = True
                i = 0
                while b:
                    cyl3 = cylindres[i]
                    if cyl3 != cyl1 and cyl3 != cyl2:
                        if distancePD(cyl3[1], droite(cyl1[1],cyl2[1])) > R:
                            g.add_edges([(str(cyl1[0]),str(cyl2[0]))])
                        else :
                            b = False
                    i += 1
                    
    g.es["Poids"] = [str(0) for k in range(len(g.get_edgelist()))]
    for i in range(len(g.get_edgelist())):
        g.es[i]["Poids"] = fct_Poids()


def exempleGraph():
    g = ig.Graph(n=5)
    g.add_edges([(0,3),(0,4),(1,3),(1,2),(2,1),(2,4),(3,0),(3,1),(3,4),(4,0),(4,3),(4,2)])
    g.vs["num"] = ["0","1","2","3","4"]
    g.vs["Masse"] = ["5","10","15","20","25"] 
    return g

g = exempleGraph()
list = g.get_edgelist()
print(list)

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