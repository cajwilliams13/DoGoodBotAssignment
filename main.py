import swift
import roboticstoolbox as rtb
from robotcore.GantryBot.GantryBot import GantryBot
from robotcore.Gripper.Gripper import Gripper
from spatialmath import SE3


def CreateEnvironment(env, transform=None):
    env.launch(realtime = True)
    #Add Everything to environment
    
    #Robots

    GanBot = GantryBot()
    #UR5Bot = rtb.models.DH.UR5()
    GripperBot = Gripper()

    #Apply Transforms
    GanBot.base = GanBot.base @ transform
    #UR5Bot.base = UR5Bot.base @ transform
    GripperBot.base = GripperBot.base @ transform

    #Add to Environment
    GanBot.add_to_env(env)
    #UR5Bot.add_to_env(env)
    GripperBot.add_to_env(env)
    

    #return (UR5Bot, UR5Bot.q, GanBot, GanBot.q)
    return


if __name__ == '__main__':
    env = swift.Swift()
    transform = SE3(0,0,0)
    #UR5Bot, UR5q, GanBot, Ganq = 
    CreateEnvironment(env, transform)
    
    #GanBot.teach(GanBot.q)
    #GanBot.test()

    input('press enter to close')


