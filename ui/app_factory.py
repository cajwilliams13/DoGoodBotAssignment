import logging

from flask import Flask, jsonify, render_template, request

logging.basicConfig(level=logging.INFO, force=True)


def create_app(task_queue, end_signal):
    app = Flask(__name__)

    @app.route('/', methods=['GET'])
    def index():
        return render_template('index.html')
    
    @app.route('/start-robot-visualiser-plot', methods=['POST'])
    def start_robot_visualiser_plot():
        task = "START_ROBOT_VISUALISER_PLOT"
        task_queue.put(task, block=True)
        task_queue_size = task_queue.qsize()
        return jsonify({'message': 'Starting robot visualiser (plot)', 'task_queue_size': task_queue_size})
    
    @app.route('/stop-robot-visualiser-plot', methods=['POST'])
    def stop_robot_visualiser_plot():
        task = "STOP_ROBOT_VISUALISER_PLOT"
        task_queue.put(task, block=True)
        task_queue_size = task_queue.qsize()
        return jsonify({'message': 'Stopping robot visualiser (plot)', 'task_queue_size': task_queue_size})
    
    @app.route('/start-robot-visualiser-3d', methods=['POST'])
    def start_robot_visualiser_3d():
        task = "START_ROBOT_VISUALISER_3D"
        task_queue.put(task, block=True)
        task_queue_size = task_queue.qsize()
        return jsonify({'message': 'Starting robot visualiser (3d)', 'task_queue_size': task_queue_size})
    
    @app.route('/stop-robot-visualiser-3d', methods=['POST'])
    def stop_robot_visualiser_3d():
        task = "STOP_ROBOT_VISUALISER_3D"
        task_queue.put(task, block=True)
        task_queue_size = task_queue.qsize()
        return jsonify({'message': 'Stopping robot visualiser (3d)', 'task_queue_size': task_queue_size})
    
    @app.route('/increment-q', methods=['POST'])
    def increment_q():
        task = "INCREMENT_Q"
        task_queue.put(task, block=True)
        task_queue_size = task_queue.qsize()
        return jsonify({'message': 'Incrementing q value', 'task_queue_size': task_queue_size})
    
    @app.route('/auto-home', methods=['POST'])
    def auto_home():
        task = "AUTO_HOME"
        task_queue.put(task, block=True)
        task_queue_size = task_queue.qsize()
        return jsonify({'message': 'Auto homing robot', 'task_queue_size': task_queue_size})
    
    @app.route('/send-robot-pose', methods=['POST'])
    def send_robot_pose():
        data = request.get_json()
        logging.info(f'Received robot pose: {data}')
        task_queue.put(data, block=True)
        return jsonify({'message': 'Received robot pose'})

    @app.route('/quit', methods=['POST'])
    def quit():
        end_signal.set()
        return jsonify({'message': 'Quitting...'})

    return app