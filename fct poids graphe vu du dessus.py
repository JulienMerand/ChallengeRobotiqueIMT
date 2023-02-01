import math
import numpy as np

Vmax0 = 1 #m.s-1
coef_av = 1/600 
coef_ac = 7.41e-5 #l.m-1.kg-1
coef_bc = 1/15  #l/m-1

def Map_vers_Cylindres(PosInit) :
    ''' Retourne la liste des cylindres avec le bon format et avec pour cylindre numero 0 la position initiale du Robo'''
    DataMap = np.loadtxt(r"C:\Users\Utilisateur\Desktop\Quentin Desmartin IMT\Challenge Robotique\donnees-map.txt", dtype=float)
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
            

def Poids_i_vers_j(Pt_i, Pt_j, pondérationTps, pondérationCarbu):
    ''' Retourne le poids du graphe vu du dessus entre le cylindre i et le cylindre j'''
    'Pt_i = [i,[Xi, Yi], Mi, Ri, type_1_2_3]'
    Mi = Pt_i[2]
    Mj = Pt_j[2]
    Rj = Pt_j[3]
    dist = math.sqrt((Pt_i[1][0]-Pt_j[1][0])**2 + (Pt_i[1][1]-Pt_j[1][1])**2)
    Cout_i_vers_j = pondérationTps * dist / Vmax0*(1-math.exp(-coef_av*Mi)) + pondérationCarbu * (coef_ac * Mi + coef_bc) * dist
    Gain_i_vers_j = Rj
    return (Cout_i_vers_j / Gain_i_vers_j)

print(Map_vers_Cylindres([3,5]))


def angle_turn(Pos_Cylindre, Pos_Robo, Orientation_Robo):
    ''' Retourne la valeur de l'angle entre le Robo et le cylindre vise'''
    # Pos_Cylindre = [Xc, Yc]
    # Pos_Robo = [Xr, Yr]
    # Orientation_Robo = angle autour de Z entre - pi et pi
    vecteur_ecart = [Pos_Cylindre[0]-Pos_Robo[0],Pos_Cylindre[1]-Pos_Robo[1]]
    vecteur_orientation = [math.cos(Orientation_Robo), math.sin(Orientation_Robo)]
    prod_scalaire = vecteur_ecart[0]*vecteur_orientation[0] + vecteur_ecart[1] + vecteur_orientation[1]
    norme_ecart = math.sqrt(Pos_Cylindre[0]**2 + Pos_Cylindre[1]**2)
    return(math.acos(prod_scalaire)/norme_ecart)