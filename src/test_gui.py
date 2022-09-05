
import os
import time
import json
import webbrowser
import threading
import cv2
import webview
import pyautogui
import numpy as np
from flask import Flask, render_template, Response, request
from test_main import MainApp

LABEL_BLUE = (217, 115, 73)

app = Flask(__name__, static_url_path='/static')
app.secret_key = "xyz"

backend = MainApp()

active_camera = 'camera1'
horizontal_flip = False
vertical_flip = False


def change_camera():
    global active_camera
    if not backend.videohandler_secondary:
        active_camera = 'camera1'
        return 'cannot change camera'
    if active_camera == 'camera1':
        active_camera = 'camera2'
        return 'display camera 2'
    if active_camera == 'camera2':
        active_camera = 'both'
        return 'display camera 1 & 2'
    if active_camera == 'both':
        active_camera = 'camera1'
        return 'display camera 1'
    return 'error'


def change_model():
    if not backend.computervision.model_name:
        return 'cannot change model'
    if backend.computervision.model_name == backend.model_manikin['name']:
        backend.computervision.load(
            backend.model_coco['name'],
            backend.model_coco['path_to_weights'],
            backend.model_coco['path_to_labels'],
            backend.model_coco['path_to_thresholds'])
        return 'Model: '+backend.model_coco['name']
    if backend.computervision.model_name == backend.model_coco['name']:
        backend.computervision.load(
            backend.model_manikin['name'],
            backend.model_manikin['path_to_weights'],
            backend.model_manikin['path_to_labels'],
            backend.model_manikin['path_to_thresholds'])
        return 'Model: '+backend.model_manikin['name']
    return 'error'


def horizontal_flip_camera():
    global horizontal_flip
    horizontal_flip = not horizontal_flip
    return 'horizontal flip'

def vertical_flip_camera():
    global vertical_flip
    vertical_flip = not vertical_flip
    return 'vertical flip'


def rounded_rectangle(src, top_left, bottom_right):

    #  corners:
    #  p1 - p2
    #  |     |
    #  p4 - p3

    p1 = top_left
    p2 = (bottom_right[0], top_left[1])
    p3 = bottom_right
    p4 = (top_left[0], bottom_right[1])

    corner_radius = 20
    # draw straight lines
    cv2.line(src, (p1[0] + corner_radius, p1[1]), (p2[0] - corner_radius, p2[1]), LABEL_BLUE, 1, cv2.LINE_AA)
    cv2.line(src, (p2[0], p2[1] + corner_radius), (p3[0], p3[1] - corner_radius), LABEL_BLUE, 1, cv2.LINE_AA)
    cv2.line(src, (p3[0] - corner_radius, p4[1]), (p4[0] + corner_radius, p3[1]), LABEL_BLUE, 1, cv2.LINE_AA)
    cv2.line(src, (p4[0], p4[1] - corner_radius), (p1[0], p1[1] + corner_radius), LABEL_BLUE, 1, cv2.LINE_AA)

    # draw arcs
    cv2.ellipse(
        src, (p1[0] + corner_radius, p1[1] + corner_radius),
        (corner_radius, corner_radius),
        180.0, 0, 90, LABEL_BLUE, 1, cv2.LINE_AA)
    cv2.ellipse(
        src, (p2[0] - corner_radius, p2[1] + corner_radius),
        (corner_radius, corner_radius),
        270.0, 0, 90, LABEL_BLUE, 1, cv2.LINE_AA)
    cv2.ellipse(
        src, (p3[0] - corner_radius, p3[1] - corner_radius),
        (corner_radius, corner_radius),
        0.0, 0, 90, LABEL_BLUE, 1, cv2.LINE_AA)
    cv2.ellipse(
        src, (p4[0] + corner_radius, p4[1] - corner_radius),
        (corner_radius, corner_radius),
        90.0, 0, 90, LABEL_BLUE, 1, cv2.LINE_AA)
    return src


def draw_boxes(frame, predictions):
    frame_with_annotation = frame
    for key, prediction in predictions.items():
        box = prediction['box']
        xmin = int(box[0])
        ymin = int(box[1])
        xmax = int(box[2])
        ymax = int(box[3])
        frame_with_annotation = rounded_rectangle(frame_with_annotation, (xmin, ymin), (xmax, ymax))
        frame_with_annotation = cv2.putText(
            frame_with_annotation, key, (xmin + 10, ymin + 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.9, LABEL_BLUE, 2, cv2.LINE_AA)
    return frame_with_annotation


def frame_generator():
    while True:
        numpyframe = None
        if active_camera in ['camera1', 'both'] and backend.new_scene and isinstance(backend.frame, np.ndarray):
            backend.new_scene = False
            numpyframe = backend.frame
            if active_camera == 'both' and isinstance(backend.videohandler_secondary.frame, np.ndarray):
                # resize second image to match width of first image
                new_width = numpyframe.shape[1]
                new_height = int(numpyframe.shape[0]/numpyframe.shape[1]*new_width)
                resized = cv2.resize(
                    backend.videohandler_secondary.frame, (new_width, new_height),
                    interpolation=cv2.INTER_LINEAR)
                numpyframe = cv2.vconcat([numpyframe, resized])
            if horizontal_flip:
                numpyframe = cv2.flip(numpyframe, 1)
            numpyframe = draw_boxes(numpyframe, backend.scene)
            if vertical_flip:
                numpyframe = cv2.flip(numpyframe, 0)
        elif active_camera == 'camera2' and backend.videohandler_secondary.frame_ready is True:
            backend.videohandler_secondary.frame_ready = False
            numpyframe = backend.videohandler_secondary.frame
            if horizontal_flip:
                numpyframe = cv2.flip(numpyframe, 1)
            if vertical_flip:
                numpyframe = cv2.flip(numpyframe, 0)
        else:
            time.sleep(0.02)

        if isinstance(numpyframe, np.ndarray):
            # convert numpy array to jpg and send it to frontend
            _, buffer = cv2.imencode('.jpg', numpyframe)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


def data_generator():
    while True:
        time.sleep(0.5)  # slow down to make it readable in the GUI
        data = {'detections': [],
                'network': [{'trained on': backend.computervision.model_name, 'type': 'FasterRCNN mobilenet v3'}],}
        for key, detection in backend.scene.items():
            score = int(detection['score']*100)
            data['detections'].append({key: score})
        datastring = json.dumps(data)
        yield f'data: {datastring}\n\n'


@app.route('/', methods=['GET'])
def main():
    thread = threading.Thread(target=backend.run, daemon=True)
    thread.start()
    return render_template('index.html')

@app.route('/video_feed', methods=['GET'])
def video_feed():
    return Response(frame_generator(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/data_feed', methods=['GET'])
def data_feed():
    return Response(data_generator(), mimetype='text/event-stream')


# @app.route('/key_down', methods=['POST'])
# def key_down():
#     data = json.loads(request.data)
#     if data['key'] == 'o':
#         backend.driver.move_translational_sled('backward')
#     if data['key'] == 'i':
#         backend.driver.move_translational_sled('forward')
#     return ('', 204)


@app.route('/key_up', methods=['POST'])
# TODO: drive device to home position with key press? e.g. hold "p" and it will drive to zero?
def key_up():
    global horizontal_flip
    global vertical_flip
    data = json.loads(request.data)
    if data['key'] == 'c':
        msg = change_camera()
        return {"message": msg}
    if data['key'] == 'm':
        msg = change_model()
        return {"message": msg}
    if data['key'] == 'h':
        msg = horizontal_flip_camera()
        return {"message": msg}
    if data['key'] == 'v':
        msg = vertical_flip_camera()
        return {"message": msg}
    if data['key'] == 'Escape':
        print('shut down')
        pyautogui.hotkey('ctrl', 'w')  # close browser tab
        os._exit(0)  # close python
    return ('', 204)


if __name__ == "__main__":

    # start GUI in own window (use in production)
    #mywindow = webview.create_window('Intubot 1.1', app, min_size=(1200, 800))
    #webview.start('', mywindow)

    # start GUI in browser (use for debugging only)
    if not os.environ.get("WERKZEUG_RUN_MAIN"):
        webbrowser.open_new('http://127.0.0.1:4001/')
        webbrowser.open_new('http://127.0.0.1:4001/')
    app.run(host="0.0.0.0", port=4001, debug=False)
