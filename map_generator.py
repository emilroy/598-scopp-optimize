import numpy as np
import cv2
from shapely.geometry import Polygon, Point
import pyproj
import os

class MapGenerator:
    def __init__(self, width=800, height=700, resolution=0.04):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.map_array = np.zeros((height, width), dtype=np.uint8)
        self.transformer = pyproj.Transformer.from_crs('epsg:4326', 'epsg:3857', always_xy=True)
    
    def convert_gps_to_pixel(self, points):
        # Convert GPS coordinates to Web Mercator
        x_meters, y_meters = zip(*[self.transformer.transform(lon, lat) for lat, lon in points])
        
        # Normalize to image coordinates
        x_min, x_max = min(x_meters), max(x_meters)
        y_min, y_max = min(y_meters), max(y_meters)
        
        # Add padding
        padding = 0.1
        x_range = x_max - x_min
        y_range = y_max - y_min
        x_min -= x_range * padding
        x_max += x_range * padding
        y_min -= y_range * padding
        y_max += y_range * padding
        
        # Scale to pixel coordinates
        x_pixels = [(x - x_min) / (x_max - x_min) * (self.width - 1) for x in x_meters]
        y_pixels = [(y - y_min) / (y_max - y_min) * (self.height - 1) for y in y_meters]
        
        return np.array(list(zip(x_pixels, y_pixels)), dtype=np.int32)

    def generate_map(self, boundary_points, geo_fencing_holes):
        # Convert boundary points to pixel coordinates
        boundary_pixels = self.convert_gps_to_pixel(boundary_points)
        
        # Fill the boundary with white (free space)
        cv2.fillPoly(self.map_array, [boundary_pixels], 254)
        
        # Draw boundary edges in black
        cv2.polylines(self.map_array, [boundary_pixels], True, 0, thickness=2)
        
        # Only process geo_fencing_holes if they exist
        if geo_fencing_holes is not None and len(geo_fencing_holes) > 0:
            for hole in geo_fencing_holes:
                hole_pixels = self.convert_gps_to_pixel(hole)
                cv2.fillPoly(self.map_array, [hole_pixels], 0)
    
    def save_pgm(self, filename):
        # Save as PGM format
        header = f"P2\n{self.width} {self.height}\n255\n"
        with open(filename, 'w') as f:
            f.write(header)
            np.savetxt(f, self.map_array, fmt='%d')
        
        # Generate corresponding YAML file
        yaml_filename = filename.replace('.pgm', '.yaml')
        with open(yaml_filename, 'w') as f:
            f.write(f"image: {os.path.basename(filename)}\n")
            f.write("mode: trinary\n")
            f.write(f"resolution: {self.resolution}\n")
            f.write("origin: [0.0, 0.0, 0.0]\n")
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.196\n")
