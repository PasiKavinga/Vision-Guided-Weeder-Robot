# weeder_control_gui_fixed_final.py
# Requirements:
#   pip install ultralytics opencv-python pyserial PySimpleGUI numpy

import time
import math
import threading
from collections import deque
from pathlib import Path
import queue

import PySimpleGUI as sg
import cv2
import serial
import serial.tools.list_ports
from ultralytics import YOLO
import numpy as np

# ---------- USER CONFIG ----------
BAUDRATE = 115200
MODEL_PATH = "best.pt"
DEFAULT_CAM_INDEX = 0
IMAGE_WIDTH = 640          # preview size
IMAGE_HEIGHT = 360
AREA_CM_X = 17.0
AREA_CM_Y = 27.0
DEFAULT_STEPS_PER_CM = 198.0
IGNORE_DISTANCE_CM = 3.0
GRIP_CLOSE_STEPS_DEFAULT = -100
LIFT_CM_AFTER_GRIP = 5.0
DEFAULT_CONF_THRESH = 0.35
INFER_W = 416
# ----------------------------------

# Global state
ser = None
serial_lock = threading.Lock()
serial_buffer = deque()      # lines from serial reader
serial_thread = None
serial_running = False

model = None
cap = None
cap_lock = threading.Lock()

preview_thread = None
preview_running = False

send_enabled = False
busy = False
ignore_list = []
ignore_lock = threading.Lock()

cal = {
    "steps_per_cm_x": DEFAULT_STEPS_PER_CM,
    "steps_per_cm_y": DEFAULT_STEPS_PER_CM,
    "steps_per_cm_z": DEFAULT_STEPS_PER_CM,
    "steps_grip_close": GRIP_CLOSE_STEPS_DEFAULT,
    "cm_per_px_x": AREA_CM_X / IMAGE_WIDTH,
    "cm_per_px_y": AREA_CM_Y / IMAGE_HEIGHT,
    "origin_corner": "top-left"
}

latest_detections = []
history = []

CONF_THRESH = DEFAULT_CONF_THRESH

# Actuation queue and worker
act_queue = queue.Queue()
act_worker_thread = None

# -------------------- helpers --------------------
def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]

def probe_cameras(max_idx=4):
    available = []
    for i in range(max_idx + 1):
        cap_try = cv2.VideoCapture(i, cv2.CAP_DSHOW if hasattr(cv2, 'CAP_DSHOW') else 0)
        if not cap_try.isOpened():
            cap_try.release()
            continue
        ret, _ = cap_try.read()
        cap_try.release()
        if ret:
            available.append(i)
        time.sleep(0.02)
    return available

# ---------------- Serial handling (single reader thread, thread-safe buffer) ----------------
def open_serial(port, window=None):
    global ser, serial_thread, serial_running
    try:
        close_serial()
        ser = serial.Serial(port, BAUDRATE, timeout=0.1)
        time.sleep(2)  # allow ESP32 reset
        clear_serial_buffer()
        serial_running = True
        serial_thread = threading.Thread(target=serial_reader, args=(window,), daemon=True)
        serial_thread.start()
        return True
    except Exception as e:
        print("Serial open error:", e)
        ser = None
        return False

def close_serial():
    global ser, serial_running
    serial_running = False
    try:
        if ser:
            ser.close()
    except:
        pass

def serial_reader(window):
    """Continuously read serial lines and push to serial_buffer and GUI log."""
    global ser, serial_running
    while serial_running:
        if ser is None or not ser.is_open:
            time.sleep(0.05)
            continue
        try:
            line = ser.readline().decode(errors='ignore').strip()
        except Exception:
            line = ""
        if line:
            with serial_lock:
                serial_buffer.append(line)
            # post to GUI
            if window is not None:
                window.write_event_value('-SERIAL-', line)
        else:
            time.sleep(0.01)

def clear_serial_buffer():
    with serial_lock:
        serial_buffer.clear()

def wait_for_serial_tokens(expected_tokens, timeout=10.0):
    """Poll serial_buffer for tokens. Returns (matched_token, full_line) or (None, None)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        with serial_lock:
            if serial_buffer:
                # iterate existing lines
                for _ in range(len(serial_buffer)):
                    line = serial_buffer.popleft()
                    for tok in expected_tokens:
                        if tok in line:
                            return tok, line
        time.sleep(0.02)
    return None, None

def send_and_wait_done(cmd, timeout=90):
    """Send a command that should produce DONE (e.g., MOVE, LOWER, LIFT_CM, GRIP_CLOSE)."""
    global ser, busy
    if ser is None:
        return False, "NO_SERIAL"
    try:
        clear_serial_buffer()
        ser.write((cmd + "\n").encode()); ser.flush()
    except Exception as e:
        return False, f"SERIAL_ERR:{e}"
    busy = True
    tok, line = wait_for_serial_tokens(["DONE", "ERROR", "PICKUP_FAIL", "PICKUP_DONE"], timeout=timeout)
    busy = False
    if tok is None:
        return False, "TIMEOUT"
    if "DONE" in tok or "PICKUP_DONE" in tok or "DONE" in line:
        return True, line
    return False, line

def send_and_wait_ok(cmd, timeout=5):
    """Send a cmd expecting OK/ERROR promptly (eg HOME, SET_SP* or JOG)."""
    global ser
    if ser is None:
        return False, "NO_SERIAL"
    try:
        clear_serial_buffer()
        ser.write((cmd + "\n").encode()); ser.flush()
    except Exception as e:
        return False, f"SERIAL_ERR:{e}"
    tok, line = wait_for_serial_tokens(["OK", "ERROR"], timeout=timeout)
    if tok is None:
        return False, "TIMEOUT"
    return (tok == "OK"), line

# -------------------- coordinate helpers --------------------
def px_to_cm_for_image(px_x, px_y, img_w, img_h):
    cm_per_px_x = AREA_CM_X / img_w
    cm_per_px_y = AREA_CM_Y / img_h
    corner = cal['origin_corner']
    if corner == "top-left":
        x_cm = px_x * cm_per_px_x
        y_cm = px_y * cm_per_px_y
    elif corner == "top-right":
        x_cm = (img_w - px_x) * cm_per_px_x
        y_cm = px_y * cm_per_px_y
    elif corner == "bottom-left":
        x_cm = px_x * cm_per_px_x
        y_cm = (img_h - px_y) * cm_per_px_y
    else:
        x_cm = (img_w - px_x) * cm_per_px_x
        y_cm = (img_h - px_y) * cm_per_px_y
    x_cm = max(0.0, min(AREA_CM_X, x_cm))
    y_cm = max(0.0, min(AREA_CM_Y, y_cm))
    return x_cm, y_cm

def px_to_cm_camera(cx_px, cy_px, cam_w, cam_h):
    x_cm = cx_px * cal.get("cm_per_px_x", AREA_CM_X / cam_w)
    y_cm = cy_px * cal.get("cm_per_px_y", AREA_CM_Y / cam_h)
    x_cm = max(0.0, min(AREA_CM_X, x_cm))
    y_cm = max(0.0, min(AREA_CM_Y, y_cm))
    return x_cm, y_cm

def is_near_ignored(x_cm, y_cm, thresh=IGNORE_DISTANCE_CM):
    with ignore_lock:
        for ix, iy in ignore_list:
            if math.hypot(x_cm-ix, y_cm-iy) <= thresh:
                return True
    return False

def add_ignore(x_cm, y_cm):
    with ignore_lock:
        ignore_list.append((x_cm, y_cm))

# -------------------- actuation worker --------------------
def actuation_worker(window):
    """Consume (x_cm, y_cm) targets and perform primitives sequentially:
       HOME/MOVE origin -> MOVE target -> LOWER -> GRIP_CLOSE -> LIFT_CM -> GRIP_OPEN -> RETURN to origin
    """
    global send_enabled
    LIFT = LIFT_CM_AFTER_GRIP
    while True:
        item = act_queue.get()
        if item is None:
            break
        x_cm, y_cm = item
        window.write_event_value('-LOG-', f"[ACT] Starting pick for {x_cm:.2f},{y_cm:.2f}")
        # 0) ensure origin 0,0 (host-level HOMING)
        ok, r = send_and_wait_ok("HOME", timeout=5)
        window.write_event_value('-LOG-', f"[ACT] HOME -> {ok},{r}")
        if not ok:
            window.write_event_value('-LOG-', "[ACT] HOME failed; trying continue")

        time.sleep(0.05)
        # 1) Move to target (MOVE is absolute: X then Y)
        # If swap checkbox enabled, swap x/y when sending
        swap_xy = window['-SWAPXY-'].get()
        if swap_xy:
            cmd_move = f"MOVE {y_cm:.2f} {x_cm:.2f}"
        else:
            cmd_move = f"MOVE {x_cm:.2f} {y_cm:.2f}"

        ok, r = send_and_wait_done(cmd_move, timeout=90)
        window.write_event_value('-LOG-', f"[ACT] {cmd_move} -> {ok},{r}")
        if not ok:
            window.write_event_value('-LOG-', "[ACT] MOVE target failed; skipping")
            continue
        time.sleep(0.05)

        # 2) Lower until switch
        ok, r = send_and_wait_done("LOWER", timeout=40)
        window.write_event_value('-LOG-', f"[ACT] LOWER -> {ok},{r}")
        if not ok:
            window.write_event_value('-LOG-', "[ACT] LOWER failed; skipping")
            continue
        time.sleep(0.05)

        # 3) Grip close
        ok, r = send_and_wait_done("GRIP_CLOSE", timeout=20)
        window.write_event_value('-LOG-', f"[ACT] GRIP_CLOSE -> {ok},{r}")
        if not ok:
            window.write_event_value('-LOG-', "[ACT] GRIP_CLOSE failed; trying continue")
        time.sleep(0.12)

        # 4) Lift by configured cm
        ok, r = send_and_wait_done(f"LIFT_CM {LIFT}", timeout=40)
        window.write_event_value('-LOG-', f"[ACT] LIFT_CM {LIFT} -> {ok},{r}")
        if not ok:
            window.write_event_value('-LOG-', "[ACT] LIFT failed; attempting continue")

        time.sleep(0.05)
        # 5) Open gripper (drop)
        ok, r = send_and_wait_done("GRIP_OPEN", timeout=20)
        window.write_event_value('-LOG-', f"[ACT] GRIP_OPEN -> {ok},{r}")

        # 6) Return to origin
        ok, r = send_and_wait_done("MOVE 0 0", timeout=90)
        window.write_event_value('-LOG-', f"[ACT] RETURN MOVE 0 0 -> {ok},{r}")

        add_ignore(x_cm, y_cm)
        history.append((x_cm, y_cm, time.strftime("%Y-%m-%d %H:%M:%S")))
        window.write_event_value('-LOG-', f"[ACT] Completed pick for {x_cm:.2f},{y_cm:.2f}")

# -------------------- detection/preview thread --------------------
def detection_loop(window):
    global cap, model, preview_running, latest_detections, send_enabled, CONF_THRESH
    preview_running = True
    infer_w = INFER_W
    while preview_running:
        with cap_lock:
            local_cap = cap
        if local_cap is None or not local_cap.isOpened():
            # blank
            blank = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8)
            cv2.putText(blank, "No camera opened", (10, IMAGE_HEIGHT//2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200,200,200), 2)
            imgbytes = cv2.imencode(".png", cv2.resize(blank, (640,360)))[1].tobytes()
            window.write_event_value('-IMAGE-', imgbytes)
            time.sleep(0.2)
            continue

        ret, frame = local_cap.read()
        if not ret:
            time.sleep(0.02); continue

        frame = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT))
        h, w = frame.shape[:2]

        detections = []
        if model is not None:
            try:
                small = cv2.resize(frame, (infer_w, int(infer_w*h/w)))
                results = model(small, imgsz=INFER_W, conf=CONF_THRESH)
            except Exception as e:
                window.write_event_value('-LOG-', f"[ERROR] Inference: {e}")
                time.sleep(0.05); continue

            scale_x = w / small.shape[1]
            scale_y = h / small.shape[0]
            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    if cls == 0:
                        x1,y1,x2,y2 = map(float, box.xyxy[0])
                        x1o = int(x1*scale_x); y1o = int(y1*scale_y)
                        x2o = int(x2*scale_x); y2o = int(y2*scale_y)
                        cx = int((x1o+x2o)/2); cy = int((y1o+y2o)/2)
                        x_cm, y_cm = px_to_cm_camera(cx, cy, w, h)
                        detections.append({"px":(x1o,y1o,x2o,y2o,cx,cy), "cm":(x_cm,y_cm), "conf": float(box.conf[0])})

        # filter ignored
        detections = [d for d in detections if not is_near_ignored(d["cm"][0], d["cm"][1])]
        latest_detections = detections

        # overlays
        for d in detections:
            x1,y1,x2,y2,cx,cy = d['px']
            cv2.rectangle(frame, (x1,y1),(x2,y2),(0,255,0),2)
            cv2.circle(frame,(cx,cy),3,(0,255,0),-1)
            xcm,ycm = d['cm']
            cv2.putText(frame, f"{xcm:.1f}cm,{ycm:.1f}cm", (cx+6, cy-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        # GUI push
        imgbytes = cv2.imencode(".png", cv2.resize(frame, (640,360)))[1].tobytes()
        window.write_event_value('-IMAGE-', imgbytes)
        window.write_event_value('-DETS-', detections)

        # auto enqueue (one at a time)
        if send_enabled and len(detections)>0 and ser is not None and not busy and act_queue.qsize()==0:
            origin = cal.get("origin_corner","top-left")
            if origin == "top-left":
                keyfunc = lambda d: (d["cm"][1], d["cm"][0])
            elif origin == "top-right":
                keyfunc = lambda d: (d["cm"][1], -d["cm"][0])
            elif origin == "bottom-left":
                keyfunc = lambda d: (-d["cm"][1], d["cm"][0])
            else:
                keyfunc = lambda d: (-d["cm"][1], -d["cm"][0])
            detections.sort(key=keyfunc)
            tx, ty = detections[0]['cm']
            act_queue.put((tx, ty))
            window.write_event_value('-LOG-', f"[DETECT] Enqueued {tx:.2f},{ty:.2f}")

        time.sleep(0.02)
    preview_running = False

# -------------------- GUI layout --------------------
sg.theme('LightBlue2')
cam_ports = probe_cameras(6)
serial_ports = list_serial_ports()

layout = [
    [sg.Column([
        [sg.Text('Camera:'), sg.Combo(cam_ports, default_value=(cam_ports[0] if cam_ports else ''), key='-CAMSEL-', size=(8,1)),
         sg.Button('Reload Cameras'), sg.Button('Open Camera'),
         sg.Text('   CONF:'), sg.Slider(range=(1,90), orientation='h', size=(20,15), default_value=int(DEFAULT_CONF_THRESH*100), key='-CONFSL-', enable_events=True)],
        [sg.Text('Serial:'), sg.Combo(serial_ports, default_value=(serial_ports[0] if serial_ports else ''), key='-PORTS-', size=(20,1)),
         sg.Button('Open Port'), sg.Button('Close Port')],
        [sg.Text('Model:'), sg.Input(MODEL_PATH, key='-MODEL-', size=(40,1)), sg.Button('Load Model')],
        [sg.Text('_'*100)],
        [sg.Image(key='-IMAGE-')],
        [sg.Button('Send Current', key='-SEND_CUR-'), sg.Button('Start', key='-START-'), sg.Button('Stop', key='-STOP-')],
        [sg.Text('Swap X/Y:'), sg.Checkbox('', key='-SWAPXY-'), sg.Text('(use if X and Y are reversed)')],
        [sg.Text('_'*100)],
    ], element_justification='left'),
     sg.Column([
        [sg.Text('Calibration (px -> cm)')],
        [sg.Text('cm_per_px_x:'), sg.Input(str(cal['cm_per_px_x']), key='-CPX-', size=(10,1)),
         sg.Text('cm_per_px_y:'), sg.Input(str(cal['cm_per_px_y']), key='-CPY-', size=(10,1))],
        [sg.Text('Steps/cm X:'), sg.Input(str(cal['steps_per_cm_x']), key='-SPX-', size=(10,1)),
         sg.Text('Steps/cm Y:'), sg.Input(str(cal['steps_per_cm_y']), key='-SPY-', size=(10,1))],
        [sg.Text('Steps/cm Z:'), sg.Input(str(cal['steps_per_cm_z']), key='-SPZ-', size=(10,1)),
         sg.Text('Grip steps:'), sg.Input(str(cal['steps_grip_close']), key='-GSP-', size=(10,1))],
        [sg.Text('Origin:'), sg.Combo(['top-left','top-right','bottom-left','bottom-right'], default_value=cal['origin_corner'], key='-ORIG-')],
        [sg.Button('Apply Calib'), sg.Button('Home (send)'), sg.Button('GRIPTEST')],
        [sg.Text('_'*40)],
        [sg.Text('Manual test & jog:')],
        [sg.Button('X -1cm'), sg.Button('X +1cm'), sg.Button('Y -1cm'), sg.Button('Y +1cm')],
        [sg.Button('Z -1cm'), sg.Button('Z +1cm')],
        [sg.Text('_'*40)],
        [sg.Text('Image file:'), sg.Input('', key='-IMGFILE-'), sg.FileBrowse(file_types=(("Image Files","*.png;*.jpg;*.jpeg"),)), sg.Button('Detect Image')],
        [sg.Text('_'*40)],
        [sg.Text('Latest Detections:'), sg.Text('History (picked):', pad=((40,0),0))],
        [sg.Listbox(values=[], size=(35,10), key='-DETLIST-'), sg.Listbox(values=[], size=(35,10), key='-HISTORY-')],
        [sg.Text('_'*40)],
        [sg.Multiline(size=(70,10), key='-LOGBOX-')]
    ])]
]

window = sg.Window("Weeder Control (final)", layout, finalize=True, resizable=True)

# ---------------- Camera helpers ----------------
def open_camera(index):
    global cap
    try:
        with cap_lock:
            if cap:
                try:
                    cap.release()
                except:
                    pass
            cam_idx = int(index)
            cap = cv2.VideoCapture(cam_idx, cv2.CAP_DSHOW if hasattr(cv2, 'CAP_DSHOW') else 0)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
            time.sleep(0.2)
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or IMAGE_WIDTH)
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or IMAGE_HEIGHT)
            window['-LOGBOX-'].print(f"Camera {cam_idx} opened: resolution {w}x{h}")
            cal['cm_per_px_x'] = AREA_CM_X / w
            cal['cm_per_px_y'] = AREA_CM_Y / h
            window['-CPX-'].update(f"{cal['cm_per_px_x']:.6f}")
            window['-CPY-'].update(f"{cal['cm_per_px_y']:.6f}")
            return True
    except Exception as e:
        window['-LOGBOX-'].print("Open camera error: " + str(e))
    return False

# Start detection thread and actuation worker
preview_thread = threading.Thread(target=detection_loop, args=(window,), daemon=True)
preview_thread.start()
act_worker_thread = threading.Thread(target=actuation_worker, args=(window,), daemon=True)
act_worker_thread.start()

# -------------------- main event loop --------------------
while True:
    event, values = window.read(timeout=100)

    if event == sg.WIN_CLOSED:
        preview_running = False
        act_queue.put(None)
        break

    if event == '-SERIAL-':
        # serial_reader pushes this event with serial line as value
        line = values['-SERIAL-']
        window['-LOGBOX-'].print("<-- " + line)

    if event == 'Reload Cameras':
        cams = probe_cameras(6)
        window['-CAMSEL-'].update(values=cams)
        window['-LOGBOX-'].print(f"Detected: {cams}")

    if event == 'Open Camera':
        sel = values['-CAMSEL-']
        if sel is None or str(sel).strip() == '':
            window['-LOGBOX-'].print("Select camera index")
        else:
            open_camera(sel)

    if event == 'Open Port':
        port = values['-PORTS-']
        if not port:
            window['-LOGBOX-'].print("Choose a serial port first")
        else:
            ok = open_serial(port, window)
            window['-LOGBOX-'].print(f"Open serial {port} -> {ok}")

    if event == 'Close Port':
        close_serial()
        window['-LOGBOX-'].print("Serial closed")

    if event == 'Load Model':
        mp = values['-MODEL-']
        try:
            model = YOLO(mp)
            window['-LOGBOX-'].print("Model loaded: " + mp)
        except Exception as e:
            model = None
            window['-LOGBOX-'].print("Model load error: " + str(e))

    if event == '-CONFSL-':
        v = float(values['-CONFSL-']) / 100.0
        CONF_THRESH = v
        window['-LOGBOX-'].print(f"Confidence set to {CONF_THRESH:.2f}")

    if event == 'Apply Calib':
        try:
            cal["cm_per_px_x"] = float(values['-CPX-'])
            cal["cm_per_px_y"] = float(values['-CPY-'])
            cal["steps_per_cm_x"] = float(values['-SPX-'])
            cal["steps_per_cm_y"] = float(values['-SPY-'])
            cal["steps_per_cm_z"] = float(values['-SPZ-'])
            cal["steps_grip_close"] = int(values['-GSP-'])
            cal["origin_corner"] = values['-ORIG-']
            window['-LOGBOX-'].print("Calibration applied")
            if ser:
                # send calibration to ESP32
                send_and_wait_ok(f"SET_SPX {cal['steps_per_cm_x']}")
                send_and_wait_ok(f"SET_SPY {cal['steps_per_cm_y']}")
                send_and_wait_ok(f"SET_SPZ {cal['steps_per_cm_z']}")
                send_and_wait_ok(f"SET_GSP {cal['steps_grip_close']}")
                # optionally set lift on ESP32
                send_and_wait_ok(f"SET_LIFT {LIFT_CM_AFTER_GRIP}")
        except Exception as e:
            window['-LOGBOX-'].print("Calibration error: " + str(e))

    if event == 'Home (send)':
        if ser:
            ok, r = send_and_wait_ok("HOME", timeout=5)
            window['-LOGBOX-'].print(f"HOME -> {ok},{r}")
        else:
            window['-LOGBOX-'].print("Open serial first")

    if event == 'GRIPTEST':
        if ser and not busy:
            ok, r = send_and_wait_done("GRIPTEST", timeout=30)
            window['-LOGBOX-'].print(f"GRIPTEST -> {ok},{r}")
        else:
            window['-LOGBOX-'].print("Serial closed or busy")

    if event == 'Start':
        if model is None:
            window['-LOGBOX-'].print("Load model first")
        elif ser is None:
            window['-LOGBOX-'].print("Open serial first")
        else:
            ok, r = send_and_wait_ok("HOME", timeout=5)
            window['-LOGBOX-'].print(f"HOME -> {ok},{r}")
            send_enabled = True
            window['-START-'].update(disabled=True)
            window['-STOP-'].update(disabled=False)
            window['-LOGBOX-'].print("Automatic picking enabled")

    if event == 'Stop':
        send_enabled = False
        window['-START-'].update(disabled=False)
        window['-STOP-'].update(disabled=True)
        window['-LOGBOX-'].print("Automatic picking disabled")

    if event == '-SEND_CUR-':
        if ser is None:
            window['-LOGBOX-'].print("Open serial first")
        elif busy:
            window['-LOGBOX-'].print("ESP32 busy, wait")
        elif len(latest_detections) == 0:
            window['-LOGBOX-'].print("No detections to send")
        else:
            tx, ty = latest_detections[0]['cm']
            # honor swap checkbox
            if window['-SWAPXY-'].get():
                act_queue.put((ty, tx))
                window['-LOGBOX-'].print(f"Manual enqueue swapped {ty:.2f},{tx:.2f}")
            else:
                act_queue.put((tx, ty))
                window['-LOGBOX-'].print(f"Manual enqueue {tx:.2f},{ty:.2f}")

    if event in ('X -1cm','X +1cm','Y -1cm','Y +1cm','Z -1cm','Z +1cm'):
        if ser and not busy:
            parts = event.split()
            axis = parts[0]
            val = int(parts[1].replace('cm',''))
            cmd = f"JOG {axis} {val}"
            ok, r = send_and_wait_ok(cmd, timeout=5)
            window['-LOGBOX-'].print(f"{cmd} -> {ok},{r}")
        else:
            window['-LOGBOX-'].print("Serial closed or busy")

    if event == 'Reload Cameras':
        cams = probe_cameras(6)
        window['-CAMSEL-'].update(values=cams)
        window['-LOGBOX-'].print(f"Detected: {cams}")

    if event == 'Detect Image':
        imgpath = values['-IMGFILE-']
        if not imgpath:
            sg.popup("Select an image file first")
        elif not Path(imgpath).exists():
            sg.popup("File not found")
        else:
            if model is None:
                window['-LOGBOX-'].print("Load model first")
            else:
                img = cv2.imread(imgpath)
                if img is None:
                    sg.popup("Unable to read image")
                else:
                    h,w = img.shape[:2]
                    scale = INFER_W / w
                    small_h = int(h*scale)
                    small = cv2.resize(img, (INFER_W, small_h))
                    try:
                        res = model(small, imgsz=INFER_W, conf=CONF_THRESH)
                    except Exception as e:
                        window['-LOGBOX-'].print("Model inference error on image: " + str(e))
                        continue
                    detections_img = []
                    sx = w / INFER_W; sy = h / small_h
                    for r in res:
                        for box in r.boxes:
                            cls = int(box.cls[0])
                            if cls == 0:
                                x1,y1,x2,y2 = map(float, box.xyxy[0])
                                x1o = int(x1*sx); y1o = int(y1*sy)
                                x2o = int(x2*sx); y2o = int(y2*sy)
                                cx = (x1o+x2o)//2; cy = (y1o+y2o)//2
                                x_cm,y_cm = px_to_cm_for_image(cx,cy,w,h)
                                detections_img.append({"px":(x1o,y1o,x2o,y2o,cx,cy),"cm":(x_cm,y_cm)})
                    if not detections_img:
                        sg.popup("No weeds detected in image")
                    else:
                        choices = [f"{i+1}: {d['cm'][0]:.2f}cm, {d['cm'][1]:.2f}cm" for i,d in enumerate(detections_img)]
                        resp = sg.popup_get_text("Detected positions:\n"+ "\n".join(choices)+ "\nEnter index to send (1..N) or Cancel")
                        try:
                            if resp:
                                idx = int(resp.strip())-1
                                tx,ty = detections_img[idx]['cm']
                                act_queue.put((tx,ty))
                                window['-LOGBOX-'].print(f"Enqueued from image: {tx:.2f},{ty:.2f}")
                        except Exception as e:
                            window['-LOGBOX-'].print("Image send error: " + str(e))

    # GUI update events
    if event == '-IMAGE-':
        img = values['-IMAGE-']
        window['-IMAGE-'].update(data=img)

    if event == '-DETS-':
        dets = values['-DETS-']
        display = []
        for d in dets:
            px = d['px'][4], d['px'][5]
            cm = d['cm']
            display.append(f"px({px[0]},{px[1]}) -> cm({cm[0]:.2f},{cm[1]:.2f}) conf:{d['conf']:.2f}")
        window['-DETLIST-'].update(display)

    if event == '-SERIAL-':
        window['-LOGBOX-'].print(values['-SERIAL-'])

    if len(history) > 0:
        hist_strings = [f"{i+1}: {h[0]:.2f},{h[1]:.2f} @ {h[2]}" for i,h in enumerate(history[-200:])]
        window['-HISTORY-'].update(hist_strings)

# cleanup
preview_running = False
act_queue.put(None)
time.sleep(0.2)
with cap_lock:
    if cap:
        cap.release()
close_serial()
window.close()
