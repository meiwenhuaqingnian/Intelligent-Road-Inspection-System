import os
import time
import json
import requests
from flask import Flask, render_template, Response, jsonify, request, send_from_directory

app = Flask(__name__)

# ==========================================
# ⚙️ 存储配置与内存状态机
# ==========================================
SAVE_DIR = "severe_cracks"
os.makedirs(SAVE_DIR, exist_ok=True)
RECORDS_FILE = "records.json" 

latest_image_bytes = None
latest_lon = None
latest_lat = None
live_frame_bytes = None
last_heartbeat_time = 0 

# 🚀 状态机与任务流水号
sys_command = "STOP"      # 默认开机待机
current_session_id = ""   # 当前巡检批次号

WEB_SERVICE_KEY = "3b56e23a509f36ab6770e5a420efa95b"

# ==========================================
# 🎮 任务中控指令接口
# ==========================================
@app.route('/api/control', methods=['POST'])
def control_device():
    """接收网页前端指令，切换系统状态"""
    global sys_command, current_session_id
    cmd = request.json.get('command')
    
    if cmd == "START":
        sys_command = "RUNNING"
        # 只要是全新启动，就生成一个新的批次号
        if not current_session_id or sys_command == "STOP":
            current_session_id = f"Task_{time.strftime('%Y%m%d_%H%M%S')}"
    elif cmd == "PAUSE":
        sys_command = "PAUSED"
    elif cmd == "STOP":
        sys_command = "STOP"
        current_session_id = "" # 结束任务
        
    return jsonify({"status": "success", "command": sys_command, "session_id": current_session_id})

@app.route('/api/command', methods=['GET'])
def get_command():
    """供树莓派轮询拉取当前最高指令"""
    return jsonify({"command": sys_command, "session_id": current_session_id})

# ==========================================
# 📡 核心数据接收接口
# ==========================================
@app.route('/upload', methods=['POST'])
def receive_upload():
    """接收策略A的高清报警图和数据"""
    global latest_image_bytes, latest_lon, latest_lat, last_heartbeat_time
    last_heartbeat_time = time.time() # 刷新看门狗
    
    lon = request.form.get('longitude', '0')
    lat = request.form.get('latitude', '0')
    severity_count = int(request.form.get('severity', '1'))
    session_id = request.form.get('session_id', '未分类任务') # 🚀 接收批次号
    
    level = "轻微" if severity_count == 1 else "中度" if severity_count == 2 else "严重"

    record = {
        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "lon": lon, "lat": lat, "level": level, "image": None,
        "session_id": session_id # 🚀 写入日志
    }

    latest_lon, latest_lat = lon, lat

    if level in ["中度", "严重"] and 'image' in request.files:
        file = request.files['image']
        image_bytes = file.read()
        latest_image_bytes = image_bytes
        
        filename = f"{level}_lon{float(lon):.6f}_lat{float(lat):.6f}_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
        save_path = os.path.join(SAVE_DIR, filename)
        with open(save_path, 'wb') as f:
            f.write(image_bytes)
        record["image"] = filename

    records = []
    if os.path.exists(RECORDS_FILE):
        with open(RECORDS_FILE, 'r', encoding='utf-8') as f:
            try: records = json.load(f)
            except: pass
    records.append(record)
    with open(RECORDS_FILE, 'w', encoding='utf-8') as f:
        json.dump(records, f, ensure_ascii=False, indent=2)

    return jsonify({"status": "success"}), 200

@app.route('/update_stream', methods=['POST'])
def update_stream():
    """接收策略B的定时心跳和坐标"""
    global live_frame_bytes, latest_lon, latest_lat, last_heartbeat_time
    last_heartbeat_time = time.time() # 刷新看门狗
    
    lon = request.form.get('longitude')
    lat = request.form.get('latitude')
    if lon and lat:
        latest_lon, latest_lat = lon, lat

    if 'frame' in request.files:
        live_frame_bytes = request.files['frame'].read()
        return "OK", 200
    return "Failed", 400

# ==========================================
# 🖥️ 网页视图及 API
# ==========================================
@app.route("/")
def home(): return render_template("home.html")
@app.route("/detection")
def detection(): return render_template("detection.html")
@app.route("/severe")
def severe(): return render_template("severe.html")

@app.route("/video_feed")
def video_feed():
    def generate():
        while True:
            if live_frame_bytes:
                yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + live_frame_bytes + b'\r\n')
            time.sleep(1.0) 
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/map_feed")
def map_feed():
    def generate_map():
        last_fetched_coords = (None, None)
        current_map_bytes = None
        while True:
            if latest_lon and latest_lat:
                if (latest_lon, latest_lat) != last_fetched_coords:
                    url = "https://restapi.amap.com/v3/staticmap"
                    params = {
                        "key": WEB_SERVICE_KEY, "location": f"{latest_lon},{latest_lat}",
                        "zoom": 16, "size": "800*600", "markers": f"mid,,A:{latest_lon},{latest_lat}"
                    }
                    try:
                        resp = requests.get(url, params=params, timeout=5)
                        if resp.status_code == 200:
                            current_map_bytes = resp.content
                            last_fetched_coords = (latest_lon, latest_lat)
                    except: pass
                if current_map_bytes:
                    yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + current_map_bytes + b'\r\n')
            time.sleep(5.0)
    return Response(generate_map(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/api/records")
def get_records():
    """供前端拉取所有 JSON 记录"""
    if os.path.exists(RECORDS_FILE):
        with open(RECORDS_FILE, 'r', encoding='utf-8') as f:
            try:
                records = json.load(f)
                records.sort(key=lambda x: x["time"], reverse=True)
                return jsonify(records)
            except: return jsonify([])
    return jsonify([])

@app.route("/api/status")
def device_status():
    """返回设备在线状态和当前任务指令"""
    is_online = (time.time() - last_heartbeat_time) < 15.0 
    return jsonify({"online": is_online, "command": sys_command, "session_id": current_session_id})

@app.route("/images/<filename>")
def serve_image(filename):
    return send_from_directory(SAVE_DIR, filename)

if __name__ == "__main__":
    import logging
    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    print("\n" + "="*50 + "\n☁️ 云端服务器(中控调度版) 已启动！监听端口: 5000\n" + "="*50 + "\n")
    app.run(host="0.0.0.0", port=5000, threaded=True)