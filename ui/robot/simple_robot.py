from roboticstoolbox import DHRobot, RevoluteDH


class SimpleRobot(DHRobot):
    def __init__(self):
        links = [
            RevoluteDH(d=0, a=1, alpha=0),
            RevoluteDH(d=0, a=1, alpha=0),
        ]
        super().__init__(links)