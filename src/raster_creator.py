from PIL import Image, ImageDraw
import math
import os
import numpy as np
import json
import cv2

# === Parametry ===
scale = 50  # 1 m = 50 px
img_w = int(40 * scale)  # 40 metrów
img_h = int(20 * scale)  # 20 metrów

# Wymiary miejsc:
# GÓRA
spacing_diag = 2.5 * math.sqrt(2) * scale  # poprawka!
length_diag = 5.0 * scale

# DÓŁ
spacing_vert = 6.0 * scale
height_vert = 3.6 * scale

# PRAWA
spacing_horiz = 2.5 * scale
length_horiz = 5.0 * scale

# Grubości linii
line_thickness = 8
border_thickness = 12

img = Image.new("L", (img_w, img_h), 255)
draw = ImageDraw.Draw(img)

# Granica mapy
draw.rectangle([0, 0, img_w - 1, img_h - 1], outline=0, width=border_thickness)

# GÓRA: skośne linie (start od góry)
start_x = int(2 * scale)
y_top = 0
for i in range(6):
    x1 = start_x + int(i * spacing_diag)
    x2 = x1 + int(length_diag * math.cos(math.radians(45)))
    y2 = y_top + int(length_diag * math.sin(math.radians(45)))
    draw.line([(x1, y_top), (x2, y2)], fill=0, width=line_thickness)

# DÓŁ: pionowe linie (od dołu w górę)
y_bottom = img_h - 1
for i in range(6):
    x = start_x + int(i * spacing_vert)
    y1 = y_bottom - int(height_vert)
    draw.line([(x, y1), (x, y_bottom)], fill=0, width=line_thickness)

# PRAWA: poziome linie (od prawej krawędzi w lewo)
x_right = img_w - 1
for i in range(6):
    y = int(2 * scale) + int(i * spacing_horiz)
    x2 = x_right - int(length_horiz)
    draw.line([(x_right, y), (x2, y)], fill=0, width=line_thickness)

# Zapis do ../data/tracks/
output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../data/tracks"))
os.makedirs(output_dir, exist_ok=True)
output_path = os.path.join(output_dir, "parking_layout.png")
img.save(output_path)

print(f"✅ Zapisano: {output_path}")


img_cv = cv2.imread(output_path, cv2.IMREAD_GRAYSCALE)
occupancy_grid = (img_cv < 128).astype(np.uint8)

# npy
occupancy_npy_path = os.path.join(output_dir, "parking_layout.npy")
np.save(occupancy_npy_path, occupancy_grid)

# json
occupancy_json_path = os.path.join(output_dir, "parking_layout.json")
with open(occupancy_json_path, "w") as f:
    json.dump(occupancy_grid.tolist(), f)

print(f"✅ Zapisano occupancy grid: {occupancy_npy_path}")
print(f"✅ Zapisano occupancy grid (json): {occupancy_json_path}")
