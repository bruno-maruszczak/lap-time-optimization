from PIL import Image, ImageDraw
import math
import os
import numpy as np
import json
import cv2

# === Parameters ===
scale = 50  # 1 m = 50 px
img_w = int(40 * scale)  # 40 meters
img_h = int(20 * scale)  # 20 meters

# Dimensions of spaces:
# TOP (angled)
desired_spacing = 2.5  # meters between lines measured perpendicularly
angle_deg = 45
angle_rad = math.radians(angle_deg)
spacing_diag = desired_spacing / math.cos(angle_rad) * scale
length_diag = 5.0 * scale

# BOTTOM (vertical)
spacing_vert = 6.0 * scale
height_vert = 3.6 * scale

# RIGHT (horizontal)
spacing_horiz = 2.5 * scale
length_horiz = 5.0 * scale

# MIDDLE (new vertical spaces)
spacing_middle = 6.0 * scale
height_middle = 3.6 * scale

# Car dimensions
car_length = 4.5 * scale
car_width = 1.9 * scale

# Line thicknesses
line_thickness = 8
border_thickness = 12

# Create image
img = Image.new("L", (img_w, img_h), 255)
draw = ImageDraw.Draw(img)

# === TOP angled lines ===
start_x = int(2 * scale)
y_top = 20
for i in range(6):
    x1 = start_x + int(i * spacing_diag)
    x2 = x1 + int(length_diag * math.cos(angle_rad))
    y2 = y_top + int(length_diag * math.sin(angle_rad))
    draw.line([(x1, y_top), (x2, y2)], fill=0, width=line_thickness)

# === BOTTOM vertical lines ===
y_bottom = img_h - 20
for i in range(6):
    x = start_x + int(i * spacing_vert)
    y1 = y_bottom - int(height_vert)
    draw.line([(x, y1), (x, y_bottom)], fill=0, width=line_thickness)

# === RIGHT horizontal lines ===
x_right = img_w - 20
for i in range(6):
    y = int(2 * scale) + int(i * spacing_horiz)
    x2 = x_right - int(length_horiz)
    draw.line([(x_right, y), (x2, y)], fill=0, width=line_thickness)

# === MIDDLE vertical lines ===
# Want to center between top and bottom:
y_middle_top = y_top + int(length_diag * math.sin(angle_rad)) + 60
y_middle_bottom = y_bottom - int(height_vert) - 60
middle_y1 = (y_middle_top + y_middle_bottom) // 2 - int(height_middle // 2)
middle_y2 = middle_y1 + int(height_middle)

for i in range(4):  # only 4 lines → 3 spaces → leaves free space on the right
    x = start_x + int(i * spacing_middle)
    draw.line([(x, middle_y1), (x, middle_y2)], fill=0, width=line_thickness)

# === Draw cars ===

# Helper to draw rotated car rectangle
def draw_car(center_x, center_y, w, h, angle_deg):
    angle_rad = math.radians(angle_deg)
    dx = w / 2
    dy = h / 2
    corners = [
        (-dx, -dy),
        (dx, -dy),
        (dx, dy),
        (-dx, dy)
    ]
    rotated_corners = []
    for (cx, cy) in corners:
        rx = center_x + cx * math.cos(angle_rad) - cy * math.sin(angle_rad)
        ry = center_y + cx * math.sin(angle_rad) + cy * math.cos(angle_rad)
        rotated_corners.append((rx, ry))
    draw.polygon(rotated_corners, fill=0)

# === Cars in BOTTOM ===
# Cars should be rotated 90 deg to fit vertical spaces
x_car_positions_bottom = [
    start_x + spacing_vert * 0 + spacing_vert / 2,
    start_x + spacing_vert * 2 + spacing_vert / 2
]
y_car_center_bottom = y_bottom - height_vert / 2
for x in x_car_positions_bottom:
    draw_car(x, y_car_center_bottom, car_width, car_length, 90)  # rotated 90°

# === Cars in RIGHT ===
y_car_positions_right = [
    int(2 * scale) + spacing_horiz * 0 + spacing_horiz / 2,
    int(2 * scale) + spacing_horiz * 2 + spacing_horiz / 2,
    int(2 * scale) + spacing_horiz * 3 + spacing_horiz / 2
]
x_car_center_right = x_right - length_horiz / 2
for y in y_car_positions_right:
    draw_car(x_car_center_right, y, car_length, car_width, 0)

# === Cars in TOP ===
# In angled spaces → compute center between lines:
for idx in [2, 4]:  # third and fifth space (idx 2 and 4)
    x1 = start_x + idx * spacing_diag
    x2 = x1 + length_diag * math.cos(angle_rad)
    y2 = y_top + length_diag * math.sin(angle_rad)
    # center point between (x1,y_top) and (x2,y2)
    center_x = (x1 + x2) / 2
    center_y = (y_top + y2) / 2
    draw_car(center_x, center_y, car_length, car_width, 45)

# === Save image and grids ===
output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../data/tracks"))
os.makedirs(output_dir, exist_ok=True)
output_path = os.path.join(output_dir, "parking_layout.png")
img.save(output_path)

print(f"Saved: {output_path}")

img_cv = cv2.imread(output_path, cv2.IMREAD_GRAYSCALE)
occupancy_grid = (img_cv < 128).astype(np.uint8)

# npy
occupancy_npy_path = os.path.join(output_dir, "parking_layout.npy")
np.save(occupancy_npy_path, occupancy_grid)

# json
occupancy_json_path = os.path.join(output_dir, "parking_layout.json")
with open(occupancy_json_path, "w") as f:
    json.dump(occupancy_grid.tolist(), f)

print(f"Saved occupancy grid: {occupancy_npy_path}")
print(f"Saved occupancy grid (json): {occupancy_json_path}")
