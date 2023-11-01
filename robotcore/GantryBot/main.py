import swift
import roboticstoolbox as rtb
from GantryBot import GantryBot



if __name__ == '__main__':
    env = swift.Swift()
    env.launch(realtime = True)
    GanBot = GantryBot()
    GanBot.add_to_env(env)
    input('press enter to close')