import osmnx as ox
import math
import networkx as nx
import re
import random
import heapq
from flask import Flask, jsonify, request
from flask_cors import CORS

# --- CẤU HÌNH ---
MAP_FILE = "phuong_tuong_mai.graphml"

print("Đang tải bản đồ...")
try:
    G = ox.load_graphml(MAP_FILE)
    print(f"Graph gốc: {len(G.nodes)} nodes.")
except Exception as e:
    print(f"Lỗi tải map: {e}. Tạo graph rỗng.")
    G = nx.MultiDiGraph()

app = Flask(__name__)
CORS(app)

# ==========================================
# QUẢN LÝ TRẠNG THÁI
# ==========================================

TRAFFIC_EDGES = {}  # {(u, v): multiplier}
FORBIDDEN_EDGES = set()
GLOBAL_RAINFALL = 0

TRAFFIC_LEVELS = {
    1: {'multiplier': 1.2, 'color': '#FFD700', 'name': 'Đông (Vàng)'},
    2: {'multiplier': 1.5, 'color': '#FF8C00', 'name': 'Tắc vừa (Cam)'},
    3: {'multiplier': 2.0, 'color': '#FF0000', 'name': 'Tắc nặng (Đỏ)'}
}

# ==========================================
# HÀM HỖ TRỢ TÍNH TOÁN & GEOMETRY
# ==========================================

def haversine_calc(lat1, lon1, lat2, lon2):
    R = 6371000 
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def parse_linestring(wkt_obj):
    """Chuyển đổi geometry object hoặc string WKT thành list toạ độ"""
    try:
        # Nếu là object shapely
        if hasattr(wkt_obj, 'coords'):
            return [(y, x) for x, y in wkt_obj.coords]
        
        # Nếu là string WKT
        wkt_str = str(wkt_obj)
        coords_text = re.search(r'\((.*?)\)', wkt_str)
        if coords_text:
            points = []
            for pair in coords_text.group(1).split(','):
                parts = pair.strip().split()
                if len(parts) >= 2:
                    points.append((float(parts[1]), float(parts[0]))) # Đảo lại thành (Lat, Lng)
            return points
    except: pass
    return []

def get_road_width(edge_data):
    width_raw = edge_data.get('width', None)
    if width_raw:
        if isinstance(width_raw, list): width_raw = width_raw[0]
        try:
            return float(re.findall(r"[-+]?\d*\.\d+|\d+", str(width_raw))[0])
        except: pass
    
    highway_type = edge_data.get('highway', 'residential')
    if isinstance(highway_type, list): highway_type = highway_type[0]
    width_mapping = {'motorway': 20, 'trunk': 15, 'primary': 12, 'secondary': 10, 'tertiary': 8}
    return width_mapping.get(highway_type, 4.0)

def get_flood_status(edge_data):
    if GLOBAL_RAINFALL <= 0: return False
    width = get_road_width(edge_data)
    # Công thức giả định: Đường càng rộng thoát nước càng tốt (hoặc ngược lại tuỳ logic)
    # Ở đây giữ nguyên logic cũ: Mưa > Width * 25 thì ngập
    capacity = width * 25 
    return GLOBAL_RAINFALL > capacity

def find_nearest_node(lat, lng):
    try:
        return ox.distance.nearest_nodes(G, X=lng, Y=lat)
    except: return None

# ==========================================
# THUẬT TOÁN A*
# ==========================================
def a_star(start_node, end_node):
    open_set = []
    heapq.heappush(open_set, (0, start_node))
    came_from = {}
    g_score = {node: float('inf') for node in G.nodes()}
    g_score[start_node] = 0
    
    while open_set:
        current_score, current = heapq.heappop(open_set)
        
        if current == end_node:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_node)
            return path[::-1]
        
        for neighbor in G.neighbors(current):
            # Check Cấm
            if (current, neighbor) in FORBIDDEN_EDGES or (neighbor, current) in FORBIDDEN_EDGES:
                continue

            # Check Ngập
            edge_data = G.get_edge_data(current, neighbor)[0]
            if get_flood_status(edge_data):
                continue 

            dist = haversine_calc(
                G.nodes[current]['y'], G.nodes[current]['x'],
                G.nodes[neighbor]['y'], G.nodes[neighbor]['x']
            )

            # Check Tắc
            traffic_factor = TRAFFIC_EDGES.get((current, neighbor), 1.0)
            
            segment_cost = dist * traffic_factor
            tentative_g_score = g_score[current] + segment_cost
            
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + haversine_calc(
                    G.nodes[neighbor]['y'], G.nodes[neighbor]['x'],
                    G.nodes[end_node]['y'], G.nodes[end_node]['x']
                )
                heapq.heappush(open_set, (f_score, neighbor))     
    return None

# ==========================================
# API HANDLERS
# ==========================================

@app.route('/api/find-path', methods=['POST'])
def find_path():
    data = request.json
    start, end = data.get('start'), data.get('end')
    try:
        u_start = find_nearest_node(start['lat'], start['lng'])
        u_end = find_nearest_node(end['lat'], end['lng'])
        if not u_start or not u_end:
            return jsonify({"status": "error", "message": "Không tìm thấy điểm"}), 404

        path_nodes = a_star(u_start, u_end)
        
        if path_nodes:
            path_coords = [(G.nodes[n]["y"], G.nodes[n]["x"]) for n in path_nodes]
            return jsonify({"status": "success", "path": path_coords})
        
        return jsonify({"status": "error", "message": "Không có đường đi (Do ngập/cấm/tắc)!"}), 404
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

def get_edges_between_points(p1, p2):
    n1 = find_nearest_node(p1['lat'], p1['lng'])
    n2 = find_nearest_node(p2['lat'], p2['lng'])
    if n1 == n2: return [], []
    try:
        path = nx.shortest_path(G, n1, n2, weight='length')
        edges = []
        for i in range(len(path) - 1):
            u, v = path[i], path[i+1]
            edges.append((u, v))
            if G.has_edge(v, u): edges.append((v, u))
        path_coords = [(G.nodes[n]['y'], G.nodes[n]['x']) for n in path]
        return edges, path_coords
    except: return [], []

@app.route('/api/add-segment', methods=['POST'])
def add_segment():
    global TRAFFIC_EDGES, FORBIDDEN_EDGES
    data = request.json
    p1, p2, mode = data.get('p1'), data.get('p2'), data.get('mode')
    level = int(data.get('level', 1))
    
    edges, path_coords = get_edges_between_points(p1, p2)
    if not edges: return jsonify({"status": "error", "message": "Không tìm thấy đường."})
    
    draw_color = '#000000'
    if mode == 'forbidden':
        for u, v in edges:
            FORBIDDEN_EDGES.add((u, v))
            TRAFFIC_EDGES.pop((u, v), None)
    elif mode == 'traffic':
        config = TRAFFIC_LEVELS.get(level, TRAFFIC_LEVELS[1])
        multiplier, draw_color = config['multiplier'], config['color']
        for u, v in edges:
            TRAFFIC_EDGES[(u, v)] = multiplier
            FORBIDDEN_EDGES.discard((u, v))
            
    return jsonify({"status": "success", "message": "Đã cập nhật.", "path": path_coords, "color": draw_color})

@app.route('/api/clear-data', methods=['POST'])
def clear_data():
    global TRAFFIC_EDGES, FORBIDDEN_EDGES
    target = request.json.get('target')
    if target == 'traffic': TRAFFIC_EDGES = {}
    elif target == 'forbidden': FORBIDDEN_EDGES = set()
    return jsonify({"status": "success", "message": "Đã xóa."})

@app.route('/api/set-rain', methods=['POST'])
def set_rain():
    global GLOBAL_RAINFALL
    GLOBAL_RAINFALL = float(request.json.get('rain', 0))
    return jsonify({"status": "success"})

# API MỚI: Lấy danh sách đường ngập
@app.route('/api/get-flooded-roads', methods=['GET'])
def get_flooded_roads():
    flooded_segments = []
    # Duyệt qua tất cả các cạnh để kiểm tra ngập
    for u, v, k, data in G.edges(keys=True, data=True):
        if get_flood_status(data):
            # Lấy geometry để vẽ cho đẹp
            if 'geometry' in data:
                coords = parse_linestring(data['geometry'])
                if coords: flooded_segments.append(coords)
            else:
                # Nếu không có geometry chi tiết thì lấy 2 điểm đầu cuối
                p1 = (G.nodes[u]['y'], G.nodes[u]['x'])
                p2 = (G.nodes[v]['y'], G.nodes[v]['x'])
                flooded_segments.append([p1, p2])
                
    return jsonify({"status": "success", "roads": flooded_segments, "rain": GLOBAL_RAINFALL})
@app.route('/api/get-nodes', methods=['GET'])
def get_nodes():
    nodes_data = []
    for node_id, data in G.nodes(data=True):
        nodes_data.append({
            'id': node_id,
            'lat': data['y'],
            'lng': data['x']
        })
    return jsonify({"status": "success", "nodes": nodes_data})
if __name__ == '__main__':
    app.run(debug=True, port=5000)
