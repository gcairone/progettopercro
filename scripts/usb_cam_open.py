import cv2
import time

cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! image/jpeg ! jpegdec ! videoconvert ! appsink")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

if not cap.isOpened():
    print("Errore nell'aprire la fotocamera")
    exit()


last_saved_time = time.time()  # Tempo dell'ultima foto salvata

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Errore nel ricevere il frame")
        break

    cv2.imshow('Camera USB', frame)


    current_time = time.time()
    if current_time - last_saved_time >= 1:  # Se è passato almeno 1 secondo
        filename = f"usb_camera_images/foto_{int(current_time)}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Foto salvata: {filename}")
        
        last_saved_time = current_time

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
