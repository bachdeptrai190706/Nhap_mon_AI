import osmnx as ox
import pandas as pd

# --- Download road network of Phường Hà Đông ---
G = ox.graph_from_place("Phường  Tương Mai, Hà Nội, Việt Nam", network_type='all')

# --- Convert to DataFrames ---
nodes, edges = ox.graph_to_gdfs(G)

# --- Save to Excel ---
with pd.ExcelWriter("phuong_tuong_mai_map.xlsx") as writer:
    nodes.to_excel(writer, sheet_name="Nodes")
    edges.to_excel(writer, sheet_name="Edges")

print("✅ Saved successfully as phuong_tuong_mai_map.xlsx")
