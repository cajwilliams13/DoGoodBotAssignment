import swift
import roboticstoolbox as rtb
from robotcore.GantryBot.GantryBot import GantryBot
from robotcore.Gripper.Gripper import Gripper
from spatialmath import SE3
from ir_support import UR5
from math import pi
import spatialgeometry as geometry


def CreateEnvironment(env, transform=None):
    env.launch(realtime = True)
    #Add Everything to environment
    
    #Objects
    #Table, Safety Stuff, Cubby, etc
    Table = geometry.Mesh('Brick.stl', pose = SE3(0,0,0))
    Extinguisher = geometry.Mesh('Brick.stl', pose = SE3(0,0,0))
    StorageUnit = geometry.Mesh('Brick.stl', pose = SE3(0,0,0))
    Printer = geometry.Mesh('Brick.stl', pose = SE3(0,0,0))
    EStop = geometry.Mesh('Brick.stl', pose = SE3(0,0,0))
    #Robots
    GanBot = GantryBot()
    UR5Bot = UR5()
    GripperBot = Gripper()

    #Apply Transforms
    GanBot.base = GanBot.base @ transform
    GripperBot.base = GripperBot.base @ transform @ SE3(0.05, 0.65, -0.33) @ SE3.Rx(pi/2)
    UR5Bot.base = UR5Bot.base @ transform @ SE3(1, 1, 0)
    

    #Add to Environment
    GanBot.add_to_env(env)
    UR5Bot.add_to_env(env)
    GripperBot.add_to_env(env)
    

    return (UR5Bot, UR5Bot.q, GanBot, GanBot.q)


if __name__ == '__main__':
    env = swift.Swift()
    transform = SE3(0,0,0)
    UR5Bot, UR5q, GanBot, Ganq = CreateEnvironment(env, transform)
    
    #GanBot.teach(GanBot.q)
    #GanBot.test()

    input('press enter to close')


