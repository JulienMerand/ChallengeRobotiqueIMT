import math

Masse_Robot = 0
Vmax0 = 1
coef_av = 1/600 


Vitesse = Vmax0
Cylindres_collectes = [0]
Recompense_collectes = 0
### for i in path :
Vitesse = Vmax0*(1-math.exp(-coef_av*Masse_Robot))
if vitesse > 0 : # and T < 600 sec
    # Go_To_Cylindre(Tab_Cylindres ,i, vitesse ):

    Cylindres_collectes.append(i)    # Recoltage du cylindre i à la fin de Go_to_Cylindre (i)
    Masse_Robot += Tab_Cylindres[Cylindres_collectes[-1]][2]
    Recompense_collectes += Tab_Cylindres[Cylindres_collectes[-1]][3]

