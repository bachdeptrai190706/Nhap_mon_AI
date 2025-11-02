# Tên file: prepare_graph.py
# (Chỉ chạy file này 1 LẦN DUY NHẤT)
import osmnx as ox

print("Đang tải bản đồ Phường Tương Mai (việc này có thể mất vài phút)...")
# Tải bản đồ từ OpenStreetMap
G = ox.graph_from_place("Phường Tương Mai, Hà Nội, Việt Nam", network_type="all")

# Lưu bản đồ đã xử lý ra file để dùng lại
ox.save_graphml(G, "phuong_tuong_mai.graphml")

print("Đã lưu bản đồ thành công! Bạn đã sẵn sàng chạy file app.py.")