# -*- coding: utf-8 -*-

import argparse
import math
import pybullet as p
from time import sleep

#Intervalle de temps entre chaque fenêtre
dt = 0.01
#Longeurs des composantes des pattes
l1 = 40
l2 = 45
l3 = 65
l4 = 87

def init():
    """Initialise le simulateur

    Retourne:
        int -- l'id du robot
    """
    # Instanciation de Bullet
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -10)

    # Chargement du sol
    planeId = p.loadURDF('plane.urdf')

    # Chargement du robot
    startPos = [0, 0, 0.1]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = p.loadURDF("./quadruped/robot.urdf",
                        startPos, startOrientation)

    p.setPhysicsEngineParameter(fixedTimeStep=dt)
    return robot

def setJoints(robot, joints):
    """Définis les angles cibles pour les moteurs du robot

    Arguments:
        int -- identifiant du robot
        joints {list} -- liste des positions cibles (rad)
    """
    jointsMap = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    for k in range(len(joints)):
        jointInfo = p.getJointInfo(robot, jointsMap[k])
        p.setJointMotorControl2(robot, jointInfo[0], p.POSITION_CONTROL, joints[k])

def steadyRobot():
    """Etablis une position stable pour le robot, posé sur
       ses 4 pattes

    Retourne:
        list -- les 12 positions cibles (radian) pour les moteurs
    """
    joints = [0]*12
    for i in [1,4,7,10]:
        joints[i] = math.pi/3
    for i in [2,5,8,11]:
        joints[i] = math.pi/1.2
    return joints

def set_angles_to_leg(leg_id, theta0, theta1, theta2, joints):
    """
    Arguments:
        leg_id -- numéro de la patte, de 0 à 3

    """
    #Modifie le tableau joints donnée en entrée
    joints[leg_id*3:leg_id*3+3] = [theta0, theta1, theta2]

def interpolate(xs, ts, t):
    """
    Arguments:
        xs -- tableau de valeurs
        ts -- tableau de valeurs
        t -- abscisse du point souhaité > 0

    Retourne:
        x -- position interpolée
    """

    if (t < ts[0] or t > ts[-1]):
            return 0
    ind = dichotomie(ts, t, [0,len(ts)])
    if(ind == len(ts) - 1):
        return xs[-1]
    m = (xs[ind+1] - xs[ind]) * 1.0/ (ts[ind+1] - ts[ind])

    return xs[ind]+m*(t-ts[ind])

def dichotomie(ts, t, a):
    """
    Arguments:
        ts -- tableau de valeurs
        t --  taleur flottante
        a -- tableau de valeurs

    Retourne:
        -- valeur flottante
    """
    if (len(ts) <= 1):
        return a[0]

    if(ts[len(ts) // 2] > t):
        return dichotomie(ts[:len(ts) // 2], t, [a[0], a[0] + len(ts) // 2])

    return dichotomie(ts[len(ts) // 2:], t, [a[0] + len(ts) // 2, a[1]])


def demo(t, amplitude):
    """Démonstration de mouvement (fait osciller une patte)

    Arguments:
        t {float} -- temps écoulé depuis le debut de la simulation

    Retourne:
        list -- les 12 positions cibles (radian) pour les moteurs
        float -- amplitude de l'oscillation
    """
    joints = steadyRobot()
    joints[0] = math.sin(t) * amplitude
    joints[1] = math.pi/3
    joints[2] = math.pi/3
    joints[3], joints[9] = -math.pi/5, math.pi/5

    return joints



def leg_ik(x,y,z):
    """Permet de contrôler la position du bout d'une seule patte
       selon les coordonnées x, y et z

    Arguments:
        x {float} -- position selon x de l'extrémité de la patte
        y {float} -- position selon y de l'extrémité de la patte
        z {float} -- position selon z de l'extrémité de la patte

    Retourne:
        list -- les 3 positions cibles (radian) pour les moteurs de la patte
    """


    joints = [0] * 3

    theta0 = math.atan2(y,x)

    aux1 = x / math.cos(theta0)

    #Calcul des cas limites, longueur de la patte au min et au max
    limit1 = l4 - l3
    limit2 = l4 + l3

    if (22 > math.sqrt(x**2 + y**2 + z**2) or math.sqrt(x**2 + y**2 + z**2) > 151):
        return joints

    aux2 = (- l4**2 + z**2 + aux1**2 + l3**2) / (2 * l3)

    theta1 = 2 * math.atan2(z + math.sqrt(aux1**2 + z**2 - aux2**2), aux1 + aux2)
    theta2 = theta1 + math.acos((aux1 - l3 * math.cos(theta1)) / l4)

    if(z > l3 * math.sin(theta1)):
        theta2 = theta1 - math.acos((aux1 - l3 * math.cos(theta1)) / l4)

    joints[0:2] = theta0, theta1, theta2
    return joints

def changement_de_repere(x,y,leg_id):
        """Permet de changer de passer du du robot (global) à
           celui de la patte leg_id (local)

        Arguments:
            x {float} -- position x globale
            y {float} -- position y globale
            leg_id {int} -- numéro de la patte, de 0 à 3

        Retourne:
            x_local -- position x locale
            y_local -- position y locale
        """

        tab_X, tab_Y = [-1, -1, 1, 1], [1, -1, -1, 1]
        aux = math.cos(math.pi / 4)
        x_local = x * aux * tab_X[leg_id] + y * aux * tab_Y[leg_id] - 50
        y_local = x * aux * tab_X[(leg_id + 1) % 4] + y * aux * tab_Y[(leg_id + 1) % 4]

        return x_local, y_local


def robot_ik(x,y,z):
        """Permet de contrôler la position du centre du robot
           selon les coordonnées x, y et z

        Arguments:
            x {float} -- position selon x du centre du robot
            y {float} -- position selon y du centre du robot
            z {float} -- position selon z du centre du robot

        Retourne:
            {liste} -- liste de toutes les positions cibles (radian) pour
                       les moteurs de chaque patte
        """

        joints = [0] * 12

        #Gestion de la patte 1
        x1, y1 = changement_de_repere(x, y, 0)
        joints[0:2] = leg_ik(-x1, -y1, -z)

        #Gestion de la patte 2
        x2, y2 = changement_de_repere(x, y, 1)
        joints[3:5] = leg_ik(-x2, -y2, -z)

        #Gestion de la patte 3
        x3, y3 = changement_de_repere(x, y, 2)
        joints[6:8] = leg_ik(-x3, -y3, -z)

        #Gestion de la patte 4
        x4, y4 = changement_de_repere(x, y, 3)
        joints[9:11] = leg_ik(-x4, -y4, -z)

        return joints


def walk(x_speed, y_speed, t_speed, t):
        """Permet de contrôler la vitesse de marche du robot en translation
           selon x et y et en rotation

        Arguments:
            x_speed {float} -- vitesse de translation selon x (m/s)
            y_speed {float} -- vitesse de translation selon y (m/s)
            t_speed {float} -- vitesse de rotation (m/s)

        Retourne:
            {liste} -- liste de toutes les positions cibles (radian) pour
                       les moteurs de chaque patte
        """
        joints = steadyRobot()

        if(y_speed != 0 or x_speed != 0 or t_speed != 0):
            cycle = 1.0
            t = t % cycle
            T = [0, cycle / 8, cycle / 4, cycle]
            Z = [-50, 20, -50, -50]
            aux = cycle * 2000 * 3 / 2
            tab_X = [-aux * x_speed, 0, aux * x_speed, -aux * x_speed]
            tab_Y = [-aux * y_speed, 0, aux * y_speed, -aux * y_speed]

            for leg_id in range(4):
                Dt = cycle / 4 * [2,0,3,1][leg_id]
                x, y, z = interpolate(tab_X, T, (t+Dt)%cycle), interpolate(tab_Y, T, (t+Dt)%cycle), interpolate(Z, T, (t+Dt)%cycle)
                x, y = changement_de_repere(x, y, leg_id)
                x+=100
                set_angles_to_leg(leg_id, leg_ik(x,y,z)[0], leg_ik(x,y,z)[1], leg_ik(x,y,z)[2], joints)

        return joints

def goto(x, y, t):
    """Permets de déplacer le robot à la position (x,y)

    Arguments:
        x {float} -- position cible
        y {float} -- position cible
        t {float} -- temps écoulé dans la simulation

    Retourne:
        {liste} -- liste de vitesses
    """
    xsens = abs(x) / x
    vitesses = [xsens * 0.005, y * 0.005 / abs(x), t]
    xs = vitesses[0]
    if (xs * t == x) or (xs * t > x):
        return [0, 0, t]
    else:
        return vitesses


if __name__ == "__main__":
    # Arguments
    parser = argparse.ArgumentParser(prog="TD Robotique S8")
    parser.add_argument('-m', type=str, help='Mode', required=True)
    parser.add_argument('-x', type=float, help='X target for goto (m)', default=1.0)
    parser.add_argument('-y', type=float, help='Y target for goto (m)', default=0.0)
    parser.add_argument('-t', type=float, help='Theta target for goto (rad)', default=0.0)
    args = parser.parse_args()

    mode, x, y, t = args.m, args.x, args.y, args.t

    if mode not in ['demo', 'leg_ik', 'robot_ik', 'walk', 'goto', 'fun']:
        print('Le mode %s est inconnu' % mode)
        exit(1)

    robot = init()
    if mode == 'demo':
        amplitude = p.addUserDebugParameter("amplitude", 0.1, 1, 0.3)
        print('Mode de démonstration...')

    elif mode == 'leg_ik':
        x = p.addUserDebugParameter("x", 20, 150, 50)
        y = p.addUserDebugParameter("y", -200, 100, 0)
        z = p.addUserDebugParameter("z", -200, 100, 0)
        print('Mode leg_ik...')

    elif mode == 'robot_ik':
        x = p.addUserDebugParameter("x", -75, 75, 0)
        y = p.addUserDebugParameter("y", -75, 75, 0)
        z = p.addUserDebugParameter("z", 0, 130, 30)
        print('Mode robot_ik...')

    elif mode == 'walk':
        x_speed = p.addUserDebugParameter("x_speed", -0.1, 0.1, 0)
        y_speed = p.addUserDebugParameter("y_speed", -0.1, 0.1, 0)
        t_speed = p.addUserDebugParameter("t_speed", -1, 1, 0)
        print('Mode walk...')

    elif mode == 'goto':
        print('Mode goto...')

    else:
        raise Exception('Mode non implemente: %s' % mode)

    t = 0

    # Boucle principale
    while True:
        t += dt

        if mode == 'demo':
            # Récupération des positions cibles
            joints = demo(t, p.readUserDebugParameter(amplitude))

        elif mode == 'leg_ik':
            joints = steadyRobot()
            #Récupération des positions cibles
            tab = leg_ik(p.readUserDebugParameter(x), p.readUserDebugParameter(y), p.readUserDebugParameter(z))
            set_angles_to_leg(2, tab[0], tab[1], tab[2], joints)

        elif mode == 'robot_ik':
            joints = steadyRobot()
            #Récupération des positions cibles
            tab = robot_ik(p.readUserDebugParameter(x), p.readUserDebugParameter(y), p.readUserDebugParameter(z))
            set_angles_to_leg(0, tab[0], tab[1], tab[2], joints)
            set_angles_to_leg(1, tab[3], tab[4], tab[5], joints)
            set_angles_to_leg(2, tab[6], tab[7], tab[8], joints)
            set_angles_to_leg(3, tab[9], tab[10], tab[11], joints)

        elif mode == 'walk':
            #Récupération des vitesses souhaitées
            joints = walk(p.readUserDebugParameter(x_speed), p.readUserDebugParameter(y_speed), p.readUserDebugParameter(t_speed), t)

        elif mode == 'goto':
            tab = goto(x,y,t)
            joints = walk(tab[0], tab[1], tab[2], t)

        # Envoi des positions cibles au simulateur
        setJoints(robot, joints)

        # Mise a jour de la simulation
        p.stepSimulation()
        sleep(dt)
