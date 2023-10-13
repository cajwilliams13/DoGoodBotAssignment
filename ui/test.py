import time

from ir_support import LinearUR5
from swift import Swift

env = Swift()
env.launch(realtime=True)

test = LinearUR5()
test.add_to_env(env)

env.close()