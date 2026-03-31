"""
道路损伤识别与定位核心模块 (边缘端智能控制版)
特性: 状态机按需拉起推理 / 模型懒加载 / 异步心跳上报
"""

import math
import os
import threading
import time
from typing import Optional, Tuple

import cv2
import requests
import numpy as np
import onnxruntime as ort

try:
    import serial
    from rpi.gps_rmc_reader import parse_rmc_from_line
    RPI_GPS_AVAILABLE = True
except ImportError:
    RPI_GPS_AVAILABLE = False
    serial = None
    parse_rmc_from_line = None


# ==========================================
# ⚙️ 第一部分：全局配置区
# ==========================================

CLOUD_SERVER_URL = "http://39.102.84.218:5000/upload"         
CLOUD_STREAM_URL = "http://39.102.84.218:5000/update_stream"  
CLOUD_COMMAND_URL = "http://39.102.84.218:5000/api/command"  
HEARTBEAT_INTERVAL = 1.0  # 🚀 优化为1秒/帧，提升中控台画面流畅度

WEB_SERVICE_KEY = os.environ.get("AMAP_KEY", "3b56e23a509f36ab6770e5a420efa95b")
MAP_FILENAME = "map_result.png"
MAP_WIDTH, MAP_HEIGHT = 800, 600
ZOOM_LEVEL = 16
ZONE_RADIUS = 20

RPI_GPS_PORT = os.environ.get("GPS_PORT", "/dev/ttyUSB0")
RPI_GPS_BAUD = int(os.environ.get("GPS_BAUD", "9600"))
RPI_GPS_TIMEOUT = 1.0
CAMERA_INDEX = int(os.environ.get("CAMERA_INDEX", "0"))
MODEL_PATH = os.environ.get("YOLO_MODEL", "best.onnx")
SAVE_DIR = os.environ.get("SAVE_DIR", "severe_cracks")

os.makedirs(SAVE_DIR, exist_ok=True)

# ==========================================
# 💾 第二部分：全局状态共享区
# ==========================================
lock = threading.Lock()

global_lon: Optional[float] = None
global_lat: Optional[float] = None
last_hardware_gps_time: float = 0.0

latest_frame: Optional[np.ndarray] = None
latest_detection_count: int = 0
latest_map: Optional[np.ndarray] = None

# 🚀 边缘端状态接收变量
global_command = "STOP" 
global_session_id = ""

# ==========================================
# 🛠️ 第三部分：基础工具与通信网络
# ==========================================

def set_location(latitude: float, longitude: float, source: str = "hardware") -> None:
    global global_lon, global_lat, last_hardware_gps_time
    with lock:
        if source == "hardware":
            global_lat, global_lon = latitude, longitude
            last_hardware_gps_time = time.time()
        elif source == "phone":
            if time.time() - last_hardware_gps_time > 5.0:
                global_lat, global_lon = latitude, longitude

def get_location() -> Tuple[Optional[float], Optional[float]]:
    with lock: return global_lon, global_lat

def lonlat_to_pixel(target_lon, target_lat, center_lon, center_lat, zoom, width, height):
    n = 2.0 ** zoom
    cx = ((center_lon + 180.0) / 360.0) * n * 256.0
    lat_rad_c = math.radians(center_lat)
    cy = (1.0 - math.log(math.tan(lat_rad_c) + (1 / math.cos(lat_rad_c))) / math.pi) / 2.0 * n * 256.0
    tx = ((target_lon + 180.0) / 360.0) * n * 256.0
    lat_rad_t = math.radians(target_lat)
    ty = (1.0 - math.log(math.tan(lat_rad_t) + (1 / math.cos(lat_rad_t))) / math.pi) / 2.0 * n * 256.0
    return int(width / 2 + (tx - cx)), int(height / 2 + (ty - cy))

def upload_to_cloud_async(frame: np.ndarray, lon: float, lat: float, severity: int) -> None:
    def _upload():
        try:
            data = {'longitude': lon, 'latitude': lat, 'severity': severity, 'timestamp': time.time(), 'session_id': global_session_id}
            if severity >= 2:
                success, encoded_image = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                if not success: return
                files = {'image': ('damage.jpg', encoded_image.tobytes(), 'image/jpeg')}
                requests.post(CLOUD_SERVER_URL, files=files, data=data, timeout=5)
            else:
                requests.post(CLOUD_SERVER_URL, data=data, timeout=5)
        except Exception: pass
    threading.Thread(target=_upload, daemon=True).start()

def cleanup_old_records(directory: str, max_files: int = 1000) -> None:
    try:
        files = [os.path.join(directory, f) for f in os.listdir(directory) if f.endswith('.jpg')]
        if len(files) > max_files:
            files.sort(key=os.path.getmtime)
            for i in range(len(files) - max_files + 10):
                if os.path.exists(files[i]): os.remove(files[i])
    except Exception: pass

# ==========================================
# 🤖 第四部分：硬件与 AI 引擎
# ==========================================

class CameraStream:
    def __init__(self, src=CAMERA_INDEX):
        self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.ret, self.frame = self.cap.read()
        self.lock = threading.Lock()
        threading.Thread(target=self.update, daemon=True).start()

    def update(self):
        while True:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.ret, self.frame = ret, frame

    def read(self):
        with self.lock: return self.ret, self.frame.copy() if self.frame is not None else None

class YOLO_ONNX_Engine:
    def __init__(self, onnx_model: str, confidence_thres: float = 0.3, iou_thres: float = 0.5):
        self.onnx_model = onnx_model
        self.confidence_thres = confidence_thres
        self.iou_thres = iou_thres
        self.classes = ["Crack", "damage", "pothole", "Pothole_water", "Pothole_water_m"]
        self.color_palette = np.random.uniform(0, 255, size=(len(self.classes), 3))

        available = ort.get_available_providers()
        providers = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in available]
        sess_options = ort.SessionOptions()
        sess_options.log_severity_level = 3
        self.session = ort.InferenceSession(self.onnx_model, sess_options=sess_options, providers=providers or available)
        self.model_inputs = self.session.get_inputs()
        self.input_width, self.input_height = 320, 320

    def preprocess(self, img: np.ndarray):
        self.img_height, self.img_width = img.shape[:2]
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        shape = img_rgb.shape[:2]
        r = min(self.input_width / shape[0], self.input_height / shape[1])
        new_unpad = round(shape[1] * r), round(shape[0] * r)
        dw, dh = (self.input_width - new_unpad[0]) / 2, (self.input_height - new_unpad[1]) / 2
        if shape[::-1] != new_unpad: img_rgb = cv2.resize(img_rgb, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = round(dh - 0.1), round(dh + 0.1)
        left, right = round(dw - 0.1), round(dw + 0.1)
        img_padded = cv2.copyMakeBorder(img_rgb, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
        image_data = np.array(img_padded) / 255.0
        image_data = np.transpose(image_data, (2, 0, 1))[None].astype(np.float32)
        return image_data, (top, left)

    def predict(self, frame: np.ndarray):
        img_data, pad = self.preprocess(frame)
        outputs = self.session.run(None, {self.model_inputs[0].name: img_data})
        output_array = np.transpose(np.squeeze(outputs[0]))
        boxes, scores, class_ids = [], [], []
        gain = min(self.input_height / self.img_height, self.input_width / self.img_width)
        output_array[:, 0] -= pad[1]
        output_array[:, 1] -= pad[0]

        for i in range(output_array.shape[0]):
            classes_scores = output_array[i][4:]
            max_score = np.amax(classes_scores)
            if max_score >= self.confidence_thres:
                class_id = np.argmax(classes_scores)
                x, y, w, h = output_array[i][0:4]
                boxes.append([int((x - w / 2) / gain), int((y - h / 2) / gain), int(w / gain), int(h / gain)])
                scores.append(max_score)
                class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(boxes, scores, self.confidence_thres, self.iou_thres)
        count = 0
        annotated_frame = frame.copy()
        if len(indices) > 0:
            for i in np.array(indices).flatten():
                x1, y1, w, h = boxes[int(i)]
                color = self.color_palette[class_ids[int(i)] % len(self.classes)]
                cv2.rectangle(annotated_frame, (x1, y1), (x1 + w, y1 + h), color, 2)
                count += 1
        return annotated_frame, count

# ==========================================
# 🔄 第五部分：核心后台工作线程
# ==========================================

def run_gps_reader() -> None:
    if not RPI_GPS_AVAILABLE: return
    try:
        ser = serial.Serial(RPI_GPS_PORT, RPI_GPS_BAUD, timeout=RPI_GPS_TIMEOUT)
        while True:
            line_bytes = ser.readline()
            if not line_bytes: continue
            fix = parse_rmc_from_line(line_bytes.decode("ascii", errors="ignore"))
            if fix and fix.status == "A" and fix.latitude_deg and fix.longitude_deg:
                set_location(fix.latitude_deg, fix.longitude_deg, source="hardware")
    except Exception: pass

def run_command_polling() -> None:
    """🚀 轮询线程：拉取云端任务指令"""
    global global_command, global_session_id
    print("[边缘端] 监听网络指令线程已启动...")
    while True:
        try:
            resp = requests.get(CLOUD_COMMAND_URL, timeout=3)
            if resp.status_code == 200:
                data = resp.json()
                global_command = data.get("command", "STOP")
                global_session_id = data.get("session_id", "")
        except Exception as e: 
            pass # 网络断开时，保持最后的状态，或者也可以强制置为 STOP
        time.sleep(2.0)

def run_heartbeat_loop() -> None:
    """🚀 心跳线程：不管是否推理，持续向云端上报当前画面与定位"""
    global latest_frame
    while True:
        time.sleep(HEARTBEAT_INTERVAL)
        with lock: frame_to_send = latest_frame.copy() if latest_frame is not None else None
        current_lon, current_lat = get_location()
        if frame_to_send is not None and current_lon is not None and current_lat is not None:
            try:
                # 压缩质量降低到 30，保证传输极快，降低带宽高压
                success, encoded_image = cv2.imencode('.jpg', frame_to_send, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
                if success:
                    files = {'frame': ('heartbeat.jpg', encoded_image.tobytes(), 'image/jpeg')}
                    data = {'longitude': current_lon, 'latitude': current_lat}
                    requests.post(CLOUD_STREAM_URL, files=files, data=data, timeout=2)
            except Exception: pass

def run_map_loop() -> None:
    global latest_map, global_lon, global_lat
    while global_lon is None or global_lat is None: time.sleep(1)
    center_lon, center_lat = global_lon, global_lat
    map_response = requests.get("https://restapi.amap.com/v3/staticmap", params={"key": WEB_SERVICE_KEY, "location": f"{center_lon},{center_lat}", "zoom": ZOOM_LEVEL, "size": f"{MAP_WIDTH}*{MAP_HEIGHT}"}, timeout=10)
    if map_response.status_code == 200:
        with open(MAP_FILENAME, "wb") as f: f.write(map_response.content)
    base_map_img = cv2.imread(MAP_FILENAME)
    zones = {}
    saved_severe_locations = set()

    while True:
        current_lon, current_lat = get_location()
        with lock:
            current_count = latest_detection_count
            frame_to_save = latest_frame.copy() if latest_frame is not None else None

        # 只有在发现损伤时，才进行地图绘制和上报异常
        if current_lon is not None and current_lat is not None and current_count > 0:
            px, py = lonlat_to_pixel(current_lon, current_lat, center_lon, center_lat, ZOOM_LEVEL, MAP_WIDTH, MAP_HEIGHT)
            if 0 <= px < MAP_WIDTH and 0 <= py < MAP_HEIGHT:
                my_zone = None
                min_dist = float("inf")
                for (cx, cy) in zones:
                    dist = math.hypot(cx - px, cy - py)
                    if dist <= ZONE_RADIUS and dist < min_dist:
                        min_dist, my_zone = dist, (cx, cy)

                if my_zone is None:
                    my_zone = (px, py)
                    zones[my_zone] = current_count
                else: zones[my_zone] = max(zones[my_zone], current_count)

                severity = zones[my_zone]
                if severity >= 1:
                    if my_zone not in saved_severe_locations and frame_to_save is not None:
                        upload_to_cloud_async(frame_to_save, current_lon, current_lat, severity)
                        if severity >= 2:
                            save_path = os.path.join(SAVE_DIR, f"severe_lon{current_lon:.6f}_lat{current_lat:.6f}_{time.strftime('%Y%m%d_%H%M%S')}.jpg")
                            cv2.imwrite(save_path, frame_to_save)
                            cleanup_old_records(SAVE_DIR)
                        saved_severe_locations.add(my_zone)
                    if severity >= 3: cv2.circle(base_map_img, my_zone, radius=12, color=(0, 0, 255), thickness=-1)
                    elif severity == 2: cv2.circle(base_map_img, my_zone, radius=9, color=(0, 165, 255), thickness=-1)
                    else: cv2.circle(base_map_img, my_zone, radius=6, color=(0, 255, 255), thickness=-1)

        with lock: latest_map = base_map_img.copy() if base_map_img is not None else None
        time.sleep(0.05)

def run_detection_loop() -> None:
    global latest_frame, latest_detection_count
    cam = CameraStream()
    model = None  # 🚀 初始时绝对不加载模型，节约内存

    print("[边缘端] 硬件初始化完成，进入待命状态...")

    while True:
        success, frame = cam.read()
        if not success or frame is None:
            time.sleep(0.01)
            continue

        # 🚀 状态机主逻辑：听令行事
        if global_command == "RUNNING":
            if model is None:
                print("[系统] 接收到执行指令，正在向内存载入 YOLO 权重...")
                model = YOLO_ONNX_Engine(MODEL_PATH)
            
            # 执行高算力 AI 推理
            annotated_frame, count = model.predict(frame)
            
        else:
            # 摸鱼待机模式：纯监控画面，无推理算力消耗
            annotated_frame = frame.copy()
            count = 0
            
            # 在画面上打上醒目的状态水印，便于排错
            status_text = "STANDBY (READY)" if global_command == "STOP" else "PAUSED"
            cv2.putText(annotated_frame, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2, cv2.LINE_AA)
            time.sleep(0.05) # 降低空转帧率，进一步降温

        with lock:
            latest_frame = annotated_frame
            latest_detection_count = count

# ==========================================
# 🚀 第六部分：系统启动主入口
# ==========================================
if __name__ == "__main__":
    import logging
    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    
    # 1. 启动硬件线程
    threading.Thread(target=run_gps_reader, daemon=True).start()

    # 2. 启动手机 GPS 注入节点
    from flask import Flask, request, jsonify
    flask_app = Flask(__name__)

    HTML_PAGE = """<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1"><title>树莓派副驾驶定位</title></head><body style="font-family: sans-serif; text-align: center; padding: 20px;"><h2>手机定位节点</h2><p>状态: <span id="status" style="color: red;">未连接</span></p><button onclick="startSync()" style="padding: 15px; font-size: 18px; background: #007bff; color: white; border: none; border-radius: 8px;">启动连续同步</button><p id="coords"></p><script>function startSync() { if ("geolocation" in navigator) { document.getElementById('status').innerText = "获取中..."; document.getElementById('status').style.color = "orange"; navigator.geolocation.watchPosition(success, error, {enableHighAccuracy: true, maximumAge: 0}); } } function success(p) { const lat = p.coords.latitude, lon = p.coords.longitude; document.getElementById('status').innerText = "✅ 正在注入"; document.getElementById('status').style.color = "green"; document.getElementById('coords').innerText = `经度: ${lon.toFixed(5)}\\n纬度: ${lat.toFixed(5)}`; fetch('/api/receive_location', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({latitude: lat, longitude: lon})}); } function error(e) { document.getElementById('status').innerText = `失败: ${e.message}`; document.getElementById('status').style.color = "red"; }</script></body></html>"""

    @flask_app.route('/')
    def index(): return HTML_PAGE
    @flask_app.route('/api/receive_location', methods=['POST'])
    def phone_gps():
        data = request.json
        if data and 'latitude' in data and 'longitude' in data:
            set_location(data['latitude'], data['longitude'], source="phone")
            return jsonify({"status": "success"})
        return jsonify({"status": "error"}), 400

    threading.Thread(target=lambda: flask_app.run(host='0.0.0.0', port=8080, debug=False, use_reloader=False, ssl_context='adhoc'), daemon=True).start()

    # 3. 启动后台服务线程
    threading.Thread(target=run_command_polling, daemon=True).start() 
    threading.Thread(target=run_map_loop, daemon=True).start()
    threading.Thread(target=run_heartbeat_loop, daemon=True).start()

    # 4. 阻塞主线程执行 AI 核心循环
    try:
        run_detection_loop()
    except KeyboardInterrupt:
        pass