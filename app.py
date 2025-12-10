import osmnx as ox
import math
import networkx as nx
import re
import random
from heapq import heappush, heappop
from flask import Flask, jsonify, request
from flask_cors import CORS

# --- CẤU HÌNH ---
MAP_FILE = "phuong_tuong_mai.graphml"

print("Đang tải bản đồ...")
try:
    G = ox.load_graphml(MAP_FILE)
    print(f"Graph gốc: {len(G.nodes)} nodes.")
except Exception as e:
    print(f"Lỗi tải map: {e}")
    G = nx.MultiDiGraph()

app = Flask(__name__)
CORS(app)

# ==========================================
# QUẢN LÝ TRẠNG THÁI
# ==========================================

USER_DATA = []          # (không còn dùng cho tắc đường theo điểm, nhưng giữ nếu sau này cần)
FORBIDDEN_DATA = []     # Danh sách điểm (lat,lng) bị cấm
GLOBAL_RAINFALL = 0     # Mưa (mm)

# Hệ số tắc đường theo level
TRAFFIC_LEVELS = {
    "light": 1.2,   # Đông
    "medium": 1.5,  # Tắc vừa
    "heavy": 2.0    # Tắc nặng
}

def haversine_calc(lat1, lon1, lat2, lon2):
    R = 6371000 
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# --- CHECK ĐƯỜNG CẤM (theo node, giữ logic cũ) ---
def is_node_forbidden(node_lat, node_lng):
    if not FORBIDDEN_DATA:
        return False
    radius = 15 
    for zone in FORBIDDEN_DATA:
        if haversine_calc(node_lat, node_lng, zone['lat'], zone['lng']) <= radius:
            return True
    return False

# --- CHECK NGẬP LỤT (giữ logic cũ) ---
# --- CẢI TIẾN HÀM CHECK CHIỀU RỘNG ---
# --- CẢI TIẾN HÀM CHECK CHIỀU RỘNG ---
def get_road_width(edge_data):
    # Lấy width từ dữ liệu map
    width_raw = edge_data.get('width', None)
    final_width = None

    if width_raw:
        # Xử lý trường hợp width là list hoặc string lạ
        if isinstance(width_raw, list):
            width_raw = str(width_raw[0])
        else:
            width_raw = str(width_raw)
        
        # Regex tìm số float đầu tiên (VD: "5.5m", "5;6", "approx 5")
        try:
            nums = re.findall(r"[-+]?\d*\.\d+|\d+", width_raw)
            if nums:
                final_width = float(nums[0])
        except Exception:
            pass

    # Nếu không parse được, dùng highway type
    if final_width is None:
        highway_type = edge_data.get('highway', 'residential')
        if isinstance(highway_type, list):
            highway_type = highway_type[0]
        
        width_mapping = {
            'motorway': 15.0, 'trunk': 12.0, 'primary': 10.0, 'secondary': 8.0, 
            'tertiary': 7.0, 'residential': 5.0, 'unclassified': 5.0,
            'service': 4.0, 'living_street': 4.0, 'footway': 2.5, 
            'path': 2.0, 'cycleway': 2.0, 'pedestrian': 2.0
        }
        final_width = width_mapping.get(highway_type, 4.0)

    # CHỐT CHẶN: Giới hạn width tối đa là 20m thôi (để dễ ngập hơn)
    if final_width > 20:
        final_width = 20.0
        
    return final_width

# --- CẢI TIẾN LOGIC TRẠNG THÁI NGẬP ---
def get_flood_status(edge_data):
    # 1. Mưa nhỏ hoặc không mưa -> Không ngập
    if GLOBAL_RAINFALL <= 10: 
        return False, 0
    
    # 2. Mưa rất to (> 1000mm) -> Ngập toàn bộ (Noah's Ark mode)
    if GLOBAL_RAINFALL > 1000:
        return True, float('inf')
    
    width = get_road_width(edge_data)
    
    # Công thức: Sức chịu đựng = width * 15 (Giảm hệ số để đường dễ ngập hơn)
    capacity = width * 15.0 
    
    if GLOBAL_RAINFALL > capacity:
        return True, float('inf')
    
    return False, 0
# --- CẢI TIẾN LOGIC NGẬP ---

# ==========================================
# GEOMETRY & A*
# ==========================================

def parse_linestring(wkt_str):
    try:
        coords_text = re.search(r'\((.*?)\)', wkt_str).group(1)
        points = []
        for pair in coords_text.split(','):
            parts = pair.strip().split()
            if len(parts) >= 2:
                points.append((float(parts[1]), float(parts[0])))
        return points
    except Exception:
        return []

def project_point_to_edge(point, edge_geom):
    min_dist = float('inf')
    proj_point = None
    for p in edge_geom:
        dist = (p[0] - point[0])**2 + (p[1] - point[1])**2
        if dist < min_dist:
            min_dist = dist
            proj_point = p
    return proj_point

def find_nearest_edge(lat, lng):
    try:
        result = ox.distance.nearest_edges(G, X=[lng], Y=[lat])
        if isinstance(result, tuple):
            u, v, key = result
        elif isinstance(result, list) or hasattr(result, '__iter__'):
            item = result[0]
            if isinstance(item, tuple) or (hasattr(item, '__len__') and len(item) == 3):
                u, v, key = item
            else:
                u, v, key = result[0], result[1], result[2]
        else:
            return None, None, None, 0
        return u, v, key, 0
    except Exception:
        return None, None, None, 0

def heuristic(graph, u, v):
    lat1, lng1 = graph.nodes[u]['y'], graph.nodes[u]['x']
    lat2, lng2 = graph.nodes[v]['y'], graph.nodes[v]['x']
    return haversine_calc(lat1, lng1, lat2, lng2)

def a_star(graph, start, goal):
    pq = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}

    while pq:
        current_prio, current = heappop(pq)
        if current == goal:
            break
        
        if current in cost_so_far and current_prio > cost_so_far[current] + heuristic(graph, current, goal):
            continue

        for neighbor in graph.neighbors(current):
            # Lấy cạnh tốt nhất giữa 2 node
            edge_key, edge_data = min(
                graph[current][neighbor].items(),
                key=lambda kv: kv[1].get('length', float('inf'))
            )
            length = float(edge_data.get('length', 1))
            traffic_factor = float(edge_data.get('traffic_factor', 1.0))

            # 1. CHECK ĐƯỜNG CẤM (CHẶN TUYỆT ĐỐI)
            node_lat = graph.nodes[neighbor]['y']
            node_lng = graph.nodes[neighbor]['x']
            if is_node_forbidden(node_lat, node_lng):
                continue 
            
            # 2. CHECK NGẬP LỤT (CHẶN TUYỆT ĐỐI - NHƯ BẠN YÊU CẦU)
            is_flooded, _ = get_flood_status(edge_data)
            if is_flooded:
                continue # <--- Dòng quan trọng: Gặp ngập là bỏ qua luôn, không xét nữa.

            # Tính chi phí bình thường
            new_cost = cost_so_far[current] + length * traffic_factor
            
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(graph, neighbor, goal)
                heappush(pq, (priority, neighbor))
                came_from[neighbor] = current

    if goal not in came_from:
        return None # Trả về None nếu không tìm được đường (do bị ngập hết lối đi)

    path = []
    curr = goal
    while curr is not None:
        path.append(curr)
        curr = came_from[curr]
    path.reverse()
    return path
def add_temp_node(graph, lat, lng, edge):
    u, v, key = edge
    temp_id = f"temp_{lat}_{lng}"
    if graph.has_node(temp_id):
        return temp_id

    graph.add_node(temp_id, y=lat, x=lng)
    
    if key in graph[u][v]:
        original_data = graph[u][v][key].copy()
    else:
        original_data = {'length': 100, 'highway': 'residential'}
        
    dist_total = float(original_data.get('length', 100))
    original_data['length'] = dist_total / 2
    
    graph.add_edge(u, temp_id, **original_data)
    graph.add_edge(temp_id, v, **original_data)
    if not original_data.get('oneway', False):
        graph.add_edge(temp_id, u, **original_data)
        graph.add_edge(v, temp_id, **original_data)
        
    return temp_id

def choose_closest_node(lat, lng, u, v):
    du = haversine_calc(lat, lng, G.nodes[u]['y'], G.nodes[u]['x'])
    dv = haversine_calc(lat, lng, G.nodes[v]['y'], G.nodes[v]['x'])
    return u if du <= dv else v

def build_path_coords(graph, nodes):
    return [(graph.nodes[n]["y"], graph.nodes[n]["x"]) for n in nodes if graph.has_node(n)]

def mark_traffic_on_path(path_nodes, level):
    factor = float(TRAFFIC_LEVELS.get(level, 1.2))
    for i in range(len(path_nodes) - 1):
        u = path_nodes[i]
        v = path_nodes[i + 1]
        if not G.has_edge(u, v):
            continue
        # Lấy cạnh ngắn nhất
        key, data = min(
            G[u][v].items(),
            key=lambda kv: kv[1].get('length', float('inf'))
        )
        current_factor = float(data.get('traffic_factor', 1.0))
        # Đoạn nặng hơn sẽ ưu tiên (max)
        data['traffic_factor'] = max(current_factor, factor)

# ==========================================
# API
# ==========================================

@app.route('/api/find-path', methods=['POST'])
def find_path():
    data = request.json
    start, end = data.get('start'), data.get('end')
    try:
        u1, v1, k1, _ = find_nearest_edge(start['lat'], start['lng'])
        u2, v2, k2, _ = find_nearest_edge(end['lat'], end['lng'])
        if u1 is None or u2 is None:
            return jsonify({"status": "error", "message": "Không tìm thấy đường gần đó"}), 404

        G_temp = G.copy()
        
        # ... (Giữ nguyên phần xử lý geometry/project point cũ) ...
        geom_start = parse_linestring(G[u1][v1][k1].get('geometry', '')) if k1 in G[u1][v1] else [(G.nodes[u1]['y'], G.nodes[u1]['x']), (G.nodes[v1]['y'], G.nodes[v1]['x'])]
        geom_end = parse_linestring(G[u2][v2][k2].get('geometry', '')) if k2 in G[u2][v2] else [(G.nodes[u2]['y'], G.nodes[u2]['x']), (G.nodes[v2]['y'], G.nodes[v2]['x'])]
        proj_start = project_point_to_edge((start['lat'], start['lng']), geom_start) or (start['lat'], start['lng'])
        proj_end = project_point_to_edge((end['lat'], end['lng']), geom_end) or (end['lat'], end['lng'])
        temp_start_id = add_temp_node(G_temp, proj_start[0], proj_start[1], (u1, v1, k1))
        temp_end_id = add_temp_node(G_temp, proj_end[0], proj_end[1], (u2, v2, k2))
        # ... (Kết thúc phần giữ nguyên) ...

        path_nodes = a_star(G_temp, temp_start_id, temp_end_id)
        
        if path_nodes:
            path_coords = [(G_temp.nodes[n]["y"], G_temp.nodes[n]["x"]) for n in path_nodes if G_temp.has_node(n)]
            return jsonify({"status": "success", "path": path_coords})
        
        # Thông báo lỗi rõ ràng khi bị chặn
        return jsonify({"status": "error", "message": "Đường bị chặn do NGẬP LỤT hoặc CẤM! Hãy chờ nước rút."}), 404
        
    except Exception as e:
        print(e)
        return jsonify({"status": "error", "message": str(e)}), 500
@app.route('/api/all-nodes', methods=['GET'])
def get_all_nodes():
    nodes = []
    for n, data in G.nodes(data=True):
        nodes.append([data['y'], data['x']])
    return jsonify({"status": "success", "count": len(nodes), "nodes": nodes})

@app.route('/api/add-traffic', methods=['POST'])
def add_traffic_segment():
    """
    Nhận 2 điểm (start, end) + level, tìm đoạn đường giữa 2 điểm đó và
    gán hệ số tắc đường cho các cạnh trên đoạn này.
    """
    data = request.json
    start = data.get('start')
    end = data.get('end')
    level = data.get('level', 'light')

    if not start or not end:
        return jsonify({"status": "error", "message": "Thiếu điểm đầu hoặc điểm cuối"}), 400

    try:
        u1, v1, k1, _ = find_nearest_edge(start['lat'], start['lng'])
        u2, v2, k2, _ = find_nearest_edge(end['lat'], end['lng'])
        if u1 is None or u2 is None:
            return jsonify({"status": "error", "message": "Không tìm thấy đường gần đó"}), 404

        start_node = choose_closest_node(start['lat'], start['lng'], u1, v1)
        end_node = choose_closest_node(end['lat'], end['lng'], u2, v2)

        path_nodes = a_star(G, start_node, end_node)
        if not path_nodes:
            return jsonify({"status": "error", "message": "Không tìm được đoạn đường giữa 2 điểm."}), 404

        # Gán hệ số tắc đường lên graph gốc
        mark_traffic_on_path(path_nodes, level)

        path_coords = build_path_coords(G, path_nodes)

        color_map = {
            "light": "#ffd000",   # vàng
            "medium": "#ff7f00",  # cam
            "heavy": "#ff0000"    # đỏ
        }
        color = color_map.get(level, "#ffd000")
        factor = TRAFFIC_LEVELS.get(level, 1.2)

        return jsonify({
            "status": "success",
            "message": "Đã đánh dấu đoạn đường tắc.",
            "level": level,
            "factor": factor,
            "color": color,
            "path": path_coords
        })
    except Exception as e:
        print(e)
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/clear-traffic', methods=['POST'])
def clear_traffic():
    """
    Xóa toàn bộ thông tin tắc đường (traffic_factor) khỏi graph.
    """
    for u, v, k, data in G.edges(keys=True, data=True):
        if 'traffic_factor' in data:
            del data['traffic_factor']
    return jsonify({"status": "success", "message": "Đã xóa tắc đường."})

@app.route('/api/set-rain', methods=['POST'])
def set_rain():
    global GLOBAL_RAINFALL
    GLOBAL_RAINFALL = float(request.json.get('rain', 0))
    return jsonify({"status": "success", "message": f"Mưa: {GLOBAL_RAINFALL}mm"})

@app.route('/api/safe-roads', methods=['GET'])
@app.route('/api/safe-roads', methods=['GET'])
def get_safe_roads():
    """
    Trả về:
      - safe_roads: các đoạn KHÔNG ngập
      - flooded_roads: các đoạn BỊ ngập
    => Frontend vẽ 2 màu khác nhau, nhìn đầy đủ mạng lưới.
    """
    safe_segments = []
    flooded_segments = []

    for u, v, k, data in G.edges(keys=True, data=True):
        is_flooded, _ = get_flood_status(data)

        # Lấy toạ độ đoạn đường
        if 'geometry' in data:
            coords = parse_linestring(data['geometry'])
            if not coords:
                continue
        else:
            p1 = (G.nodes[u]['y'], G.nodes[u]['x'])
            p2 = (G.nodes[v]['y'], G.nodes[v]['x'])
            coords = [p1, p2]

        if is_flooded:
            flooded_segments.append(coords)
        else:
            safe_segments.append(coords)

    return jsonify({
        "status": "success",
        "safe_roads": safe_segments,
        "flooded_roads": flooded_segments
    })
@app.route('/api/add-forbidden', methods=['POST'])
def add_forbidden():
    """
    Nhận 2 điểm (start, end), tìm đoạn đường giữa 2 điểm đó
    rồi đánh dấu tất cả các node trên đoạn đó là CẤM (FORBIDDEN_DATA).
    """
    global FORBIDDEN_DATA
    data = request.json
    start = data.get('start')
    end = data.get('end')

    if not start or not end:
        return jsonify({"status": "error", "message": "Thiếu điểm đầu hoặc điểm cuối"}), 400

    try:
        u1, v1, k1, _ = find_nearest_edge(start['lat'], start['lng'])
        u2, v2, k2, _ = find_nearest_edge(end['lat'], end['lng'])
        if u1 is None or u2 is None:
            return jsonify({"status": "error", "message": "Không tìm thấy đường gần đó"}), 404

        start_node = choose_closest_node(start['lat'], start['lng'], u1, v1)
        end_node = choose_closest_node(end['lat'], end['lng'], u2, v2)

        path_nodes = a_star(G, start_node, end_node)
        if not path_nodes:
            return jsonify({"status": "error", "message": "Không tìm được đoạn đường giữa 2 điểm."}), 404

        for n in path_nodes:
            FORBIDDEN_DATA.append({'lat': G.nodes[n]['y'], 'lng': G.nodes[n]['x']})

        path_coords = build_path_coords(G, path_nodes)

        return jsonify({
            "status": "success",
            "message": "Đã chặn đường.",
            "path": path_coords
        })
    except Exception as e:
        print(e)
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/api/clear-forbidden', methods=['POST'])
def clear_forbidden():
    global FORBIDDEN_DATA
    FORBIDDEN_DATA = []
    return jsonify({"status": "success", "message": "Đã gỡ lệnh cấm."})

if __name__ == '__main__':
    app.run(debug=True, port=5000)
