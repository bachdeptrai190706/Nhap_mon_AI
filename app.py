# Tên file: app.py
# (Phiên bản cập nhật - Tìm đường giữa các ngã giao ngẫu nhiên)

import osmnx as ox
import math
import random # Thêm thư viện random
from heapq import heappush, heappop
from flask import Flask, jsonify
from flask_cors import CORS

print("Bắt đầu khởi động server...")

# === TẢI BẢN ĐỒ ===
print("Đang tải file bản đồ phuong_tuong_mai.graphml...")
G = ox.load_graphml("phuong_tuong_mai.graphml")

# === LỌC RA CÁC NGÃ GIAO ===
print("Đang lọc ra các ngã giao (junctions)...")
# Lấy "degree" (số cạnh kết nối) của mỗi nút
degrees = G.degree()
# Ngã giao là nút có 1 cạnh (ngõ cụt) hoặc 3+ cạnh (ngã ba, ngã tư...)
# Chúng ta loại bỏ các nút có 2 cạnh (chỉ là điểm uốn trên đường)
junctions = [node for node, degree in degrees if degree != 2]
print(f"Đã tìm thấy {len(junctions)} ngã giao. Server đã sẵn sàng!")


# === KHỞI TẠO FLASK (Web Server) ===
app = Flask(__name__)
CORS(app) # Cho phép web front-end của bạn gọi API này

# === SAO CHÉP HÀM A* VÀ HAVERSINE CỦA BẠN VÀO ĐÂY ===
def haversine(node1, node2):
    lat1, lon1 = G.nodes[node1]["y"], G.nodes[node1]["x"]
    lat2, lon2 = G.nodes[node2]["y"], G.nodes[node2]["x"]
    R = 6371e3
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def a_star(graph, start, goal):
    open_set = []
    heappush(open_set, (0, start))
    came_from = {}
    g_score = {n: float("inf") for n in graph.nodes}
    g_score[start] = 0
    f_score = {n: float("inf") for n in graph.nodes}
    f_score[start] = haversine(start, goal)

    while open_set:
        _, current = heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1] # Trả về đường đi

        for neighbor in graph.neighbors(current):
            edge_data = graph[current][neighbor][0]
            weight = edge_data.get("length", haversine(current, neighbor))
            tentative_g = g_score[current] + weight
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + haversine(neighbor, goal)
                heappush(open_set, (f_score[neighbor], neighbor))
    return None # Không tìm thấy đường

# === TẠO ĐƯỜNG LINK API MỚI ===
# API này dùng phương thức GET (vì không cần gửi dữ liệu lên)
@app.route('/api/random-junction-path', methods=['GET'])
def find_random_path_api():
    try:
        # 1. Chọn 2 ngã giao ngẫu nhiên từ danh sách đã lọc
        start_node = random.choice(junctions)
        end_node = random.choice(junctions)

        # Đảm bảo 2 điểm không trùng nhau
        while start_node == end_node:
            end_node = random.choice(junctions)

        # 2. Chạy thuật toán A* của bạn
        path_nodes = a_star(G, start_node, end_node)

        if path_nodes:
            # 3. Chuyển đổi NÚT (node) thành TỌA ĐỘ (lat, lon)
            path_coords = [(G.nodes[n]["y"], G.nodes[n]["x"]) for n in path_nodes]
            # Lấy tọa độ của điểm đầu và cuối để vẽ marker
            start_coord = (G.nodes[start_node]["y"], G.nodes[start_node]["x"])
            end_coord = (G.nodes[end_node]["y"], G.nodes[end_node]["x"])
            
            # 4. Trả về kết quả (JSON) cho front-end
            return jsonify({
                "status": "success",
                "path": path_coords,
                "start_point": start_coord,
                "end_point": end_coord
            })
        else:
            return jsonify({"status": "error", "message": "Không tìm thấy đường đi"}), 404

    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

# === CHẠY SERVER ===
if __name__ == '__main__':
    app.run(debug=True, port=5000)