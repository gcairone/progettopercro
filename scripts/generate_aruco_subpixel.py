import cv2
import numpy as np
from fpdf import FPDF

def generate_aruco_marker(id, dict_type=cv2.aruco.DICT_6X6_250, marker_size=500, corner_size=50):
    aruco_dict = cv2.aruco.Dictionary_get(dict_type)
    marker = np.ones((marker_size, marker_size), dtype=np.uint8) * 255  
    marker_black = cv2.aruco.drawMarker(aruco_dict, id, marker_size - 2 * corner_size)  
    
    start = corner_size
    marker[start:start + marker_black.shape[0], start:start + marker_black.shape[1]] = marker_black
    
    marker_with_corners = add_corner_squares(marker, corner_size)
    
    return marker_with_corners

def add_corner_squares(marker, corner_size):
    marker_size = marker.shape[0]
    
    cv2.rectangle(marker, (0, 0), (corner_size, corner_size), 0, -1)  # Top-left
    cv2.rectangle(marker, (marker_size - corner_size, 0), (marker_size, corner_size), 0, -1)  # Top-right
    cv2.rectangle(marker, (0, marker_size - corner_size), (corner_size, marker_size), 0, -1)  # Bottom-left
    cv2.rectangle(marker, (marker_size - corner_size, marker_size - corner_size), (marker_size, marker_size), 0, -1)  # Bottom-right
    
    return marker

def create_pdf(num_markers=100, marker_size_mm=200, corner_size_mm=20, dpi=300):
    pdf = FPDF()
    pdf.set_auto_page_break(auto=True, margin=0)
    pdf.add_page()  
    
    page_width = 210  # A4 width in mm
    page_height = 297  # A4 height in mm
    
    px_per_mm = dpi / 25.4
    marker_size_px = int(marker_size_mm * px_per_mm)
    corner_size_px = int(corner_size_mm * px_per_mm)
    
    for i in range(num_markers):
        if i > 0 and i % 1 == 0:
            pdf.add_page()
        
        x_offset = 5
        y_offset = 5
        
        marker_img = generate_aruco_marker(i, marker_size=marker_size_px, corner_size=corner_size_px)
        
        marker_filename = f"marker_{i}.png"
        cv2.imwrite(marker_filename, marker_img)
        
        pdf.image(marker_filename, x=x_offset, y=y_offset, w=marker_size_mm, h=marker_size_mm)
        
        label_x = x_offset 
        label_y = y_offset + marker_size_mm + 5 
        pdf.set_font("Arial", size=10)
        pdf.text(label_x, label_y, str(i)) 
    
    pdf.output("aruco_markers.pdf")
    print("PDF con 100 marker ArUco creato con successo!")

create_pdf()
