import logging
import multiprocessing
import os
import queue
import signal
import time

from ir_support import LinearUR5
from swift import Swift

from app_factory import create_app
from robot.simple_robot import SimpleRobot
from threads.implementations.robot_controller_thread import \
    RobotControllerThread


def create_app_and_listen(task_queue, end_signal):
    app = create_app(task_queue, end_signal)
    app.run(host="127.0.0.1", port=8080, debug=False)

def create_robot_and_plot(task_queue, end_signal):
    # robot = SimpleRobot()
    robot = LinearUR5()
    robot_task_queue = queue.Queue()
    robot_controller_thread = RobotControllerThread(robot, robot_task_queue)
    robot_controller_thread.start()

    plot = None
    swift_env = None

    while True:
        if end_signal.is_set():
            if plot is not None:
                plot.close()
                plot = None
            if swift_env is not None:
                swift_env.close()
                swift_env = None
            break
        try:
            latest_task = task_queue.get(block=False)
        except queue.Empty:
            latest_task = None
        
        if latest_task == "START_ROBOT_VISUALISER_PLOT":
            if plot is None:
                plot = robot.plot(robot.q)
            else:
                logging.info('Robot visualiser already created')
        elif latest_task == "STOP_ROBOT_VISUALISER_PLOT":
            if plot is not None:
                plot.close()
                plot = None
            else:
                logging.info('Robot visualiser already stopped')
        elif latest_task == "INCREMENT_Q":
            robot_task_queue.put("INCREMENT_Q", block=True)
        elif latest_task == "AUTO_HOME":
            robot_task_queue.put("AUTO_HOME", block=True)
        elif latest_task == "START_ROBOT_VISUALISER_3D":
            if swift_env is None:
                logging.info('Creating swift environment')
                swift_env = Swift()
                swift_env.launch(realtime=True)
                robot.add_to_env(swift_env)
        elif latest_task == "STOP_ROBOT_VISUALISER_3D":
            if swift_env is not None:
                logging.info('Closing swift environment')
                swift_env.close()
                swift_env = None
            else:
                logging.info('Swift environment already closed')
        elif isinstance(latest_task, dict):
            logging.info(f'Received robot pose (main.py): {latest_task}')
            # Itterate over all the keys of the dict and turn them into floats
            for key in latest_task.keys():
                latest_task[key] = float(latest_task[key])
            robot_task = dict(x=latest_task['x'], y=latest_task['y'], z=latest_task['z'])
            robot_task_queue.put(robot_task, block=True)

        if plot is not None:
            plot.step()
        if swift_env is not None:
            swift_env.step()



if __name__ == "__main__":
    task_queue = multiprocessing.Queue()
    end_signal = multiprocessing.Event()

    p1 = multiprocessing.Process(target=create_app_and_listen, args=(task_queue,end_signal,))
    p2 = multiprocessing.Process(target=create_robot_and_plot, args=(task_queue,end_signal,))

    p1.start()
    p2.start()

    original_handler = signal.getsignal(signal.SIGINT)

    def sigint_handler(signum, frame):
        logging.info('SIGINT received. Stopping threads...')
        end_signal.set()

        if p1.is_alive():
            p1.terminate()
            p1.join()
        if p2.is_alive():
            p2.terminate()
            p2.join()
        original_handler(signum, frame)

    try:
        signal.signal(signal.SIGINT, sigint_handler)
    except ValueError as e:
        logging.error(f'{e}. Continuing execution...')

    while True:
        # Check if the end signal has been set
        if end_signal.is_set():
            p1.terminate()
            p1.join()
            p2.terminate()
            p2.join()
            break
        time.sleep(0.1)
