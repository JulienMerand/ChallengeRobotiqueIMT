import math

Vmax0 = 20
coef_a = 10
coef_ac = 10
coef_bc = 10


def Poids_i_vers_j(Pt_i, Pt_j, pondérationTps, pondérationCarbu):
    ''' Retourne le poids du graphe vu du dessus entre le cylindre i et le cylindre j'''
    'Pt_i = [i,[Xi, Yi], Mi, Ri]'
    Mi = Pt_i[2]
    Mj = Pt_j[2]
    Rj = Pt_j[3]
    dist = math.sqrt((Pt_i[1][0]-Pt_j[1][0])**2 + (Pt_i[1][1]-Pt_j[1][1])**2)
    Cout_i_vers_j = pondérationTps * dist / Vmax0*(1-math.exp(-coef_a*Mi)) + pondérationCarbu * (coef_ac * Mi + coef_bc) * dist
    Gain_i_vers_j = Rj
    return (Cout_i_vers_j / Gain_i_vers_j)
