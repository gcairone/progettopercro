import cv2
import numpy as np
import glob
import yaml
import random 
chessboard_size = (5, 7)  
square_size = 0.04375

objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size

tobjpoints = []  
timgpoints = []  

images = glob.glob('camera_images/*.jpg')
random.shuffle(images)
images = images[:50]
gray = None
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 40, 0.001)
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    try:
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, 1), criteria)
    except:
        print(f"skipping image {fname}")
        continue
    
    if ret:
        tobjpoints.append(objp)
        timgpoints.append(corners)
        
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('Chessboard', img)
        cv2.waitKey(5)

cv2.destroyAllWindows()
flags = cv2.CALIB_FIX_ASPECT_RATIO or cv2.CALIB_RATIONAL_MODEL
ret, mtx, dist, rvecs, tvecs, stdi, stde, pve = cv2.calibrateCameraExtended(
    tobjpoints, 
    timgpoints, 
    gray.shape[::-1], 
    None,
    None,
    flags=flags
)

data = {
    'camera_matrix': mtx.tolist(),
    'dist_coeffs': dist.tolist()
}

with open('camera_calibration.yaml', 'w') as f:
    yaml.dump(data, f)

print("Calibrazione completata!")
print("Matrice della fotocamera:\n", mtx)
print("Coefficiente di distorsione:\n", dist)
print("STD su intrinseci:\n", stdi)
# print("STD su estrinseci:\n", stde.mean())
print("Per View Error:\n", pve.mean())
