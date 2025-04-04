import cv2
import numpy as np
from fpdf import FPDF

def generate_aruco_marker(id, dict_type=cv2.aruco.DICT_4X4_50):
    aruco_dict = cv2.aruco.Dictionary_get(dict_type)
    marker = cv2.aruco.drawMarker(aruco_dict, id, 500)  # 500px è la dimensione dell'immagine del marker
    return marker


def create_pdf(num_markers=50):
    pdf = FPDF()
    pdf.set_auto_page_break(auto=True, margin=15)
    pdf.add_page()
    
    page_width = 210  # A4 width in mm
    page_height = 297  # A4 height in mm
    marker_size = 100  # Dimensione del marker (in mm)
    
    for i in range(num_markers):
        if i > 0 and i % 1 == 0:
            pdf.add_page()

        x_offset = (page_width - marker_size) / 2
        y_offset = (page_height - marker_size) / 2
        
        marker_img = generate_aruco_marker(i)
        
        marker_filename = f"marker_{i}.png"
        cv2.imwrite(marker_filename, marker_img)
        
        pdf.image(marker_filename, x=x_offset, y=y_offset, w=marker_size, h=marker_size)

        label_x = x_offset 
        label_y = y_offset + marker_size + 5 
        pdf.set_font("Arial", size=10)
        pdf.text(label_x, label_y, str(i))  # Aggiungi l'ID del marker come testo

    pdf.output("aruco_markers.pdf")
    print("PDF con 50 marker ArUco creato con successo!")

create_pdf()
