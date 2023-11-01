import swift
import roboticstoolbox as rtb
from robotcore.GantryBot.GantryBot import GantryBot
from robotcore.UR3Lin.UR3Lin import UR3Lin


if __name__ == '__main__':
    env = swift.Swift()
    env.launch(realtime = True)
    GanBot = UR3Lin()
    GanBot.add_to_env(env)
    input('press enter to close')