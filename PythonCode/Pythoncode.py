import serial
import math
import open3d as o3d
import numpy as np

#Constants
ROTATIONS = 3 #number of rotations
ANGLE = 11.25 #angle of each step in rotation

#Variables 
distance_m = [] #stores measurement data
map = [] #2d map array with x y z format

s = serial.Serial('COM4', 115200) 

s.write(b'e')

while(True):
    line = s.readline()
    if line.decode('utf-8').strip() == 'x':
        break
    if line:
        distance_m.append(line.decode('utf-8').strip())

s.close()


z = 100 
counter = 0

for i in range(len(distance_m)): 
    x = float(distance_m[i]) * math.sin(math.radians(ANGLE)*(i%(len(distance_m)/ROTATIONS)))  
    y = float(distance_m[i]) * math.cos(math.radians(ANGLE)*(i%(len(distance_m)/ROTATIONS)))  

    if (i) % (len(distance_m) / ROTATIONS) == 0: 
        z += 600
    map.append([x, y, z]) 

    counter = counter + 1

points = np.array(map)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
kdtree = o3d.geometry.KDTreeFlann(pcd)
k = 15 
lines = []

for i in range(len(points)):
    [k, idx, _] = kdtree.search_knn_vector_3d(pcd.points[i], k)
    for j in range(1, k):  
        lines.append([i, idx[j]])
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(lines)

window_size = (1280, 720) 
o3d.visualization.draw_geometries([pcd, line_set], window_name="Open3D", width=window_size[0], height=window_size[1])