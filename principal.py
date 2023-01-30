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
    w = DEG2RAD(math.fabs(dbVel))
    dT = DEG2RAD(math.fabs(dbAngle)) / w
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
    ''' Dist = m / Vel = m/s'''
    #......
    # init 
    #......
    w = math.fabs(dbVel)
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

###########################################################################################################################################################
# INIT
###########################################################################################################################################################

argc = len(sys.argv)
if( argc == 1 ):
  print(sys.argv[0] + '< Adresse IP du serveur V-REP>')
  exit()
#.........................................
# tentative de connexion au serveur V-REP 
#.........................................
vrep.simxFinish(-1)
time.sleep(1)

siID = vrep.simxStart(sys.argv[1], VREP_PORT, 1, 1, 10000, 50)

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

Go(10,4)
Turn(90, 90)
Go(10, 4)
Turn(90, 90)
Go(10, 4)
Turn(90, 90)
Go(10, 4)
Turn(90, 90)


###########################################################################################################################################################
# DECONNECTION DU SERVEUR
###########################################################################################################################################################

time.sleep(1)
vrep.simxFinish(siID)

