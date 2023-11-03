import logging

import swift
from spatialmath import SE3

from e_stop.e_stop import EStop
from logger.setup_logger import setup_logger


def main():
    setup_logger()

    env = swift.Swift()
    env.launch(realtime = True)
    
    estop = EStop(SE3(0,0,0), use_physical_button=True)
    print("test")
    estop.add_to_env(env)
    

if __name__ == '__main__':
    main()