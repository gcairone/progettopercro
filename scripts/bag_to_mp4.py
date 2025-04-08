#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoRecorder:
    def __init__(self):
        rospy.init_node("video_recorder", anonymous=True)
        self.bridge = CvBridge()

        # Parametri di salvataggio del video
        self.video_filename = "output_video.mp4"
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec per MP4
        self.frame_rate = 30  # Frame rate del video
        self.frame_size = (720, 540)  # Risoluzione del video (adatta alla tua telecamera)
        
        # Inizializza il VideoWriter
        self.video_writer = cv2.VideoWriter(self.video_filename, self.fourcc, self.frame_rate, self.frame_size)

        # Sottoscrizione al topic dell'immagine
        self.image_sub = rospy.Subscriber("/alphasense_driver_ros/cam2", Image, self.image_callback)

        rospy.loginfo("Registrazione video iniziata...")

    def image_callback(self, msg):
        try:
            # Converte il messaggio Image in un'immagine OpenCV (BGR)
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"Errore nella conversione dell'immagine: {e}")
            return

        # Ridimensiona il frame per il video (assicurati che la risoluzione corrisponda a quella del tuo video)
        frame_resized = cv2.resize(frame, self.frame_size)

        # Scrivi il frame nel file video
        self.video_writer.write(frame_resized)

        # Visualizza il frame in tempo reale (opzionale)
        cv2.imshow("Recording", frame_resized)
        cv2.waitKey(1)

    def stop_recording(self):
        rospy.loginfo("Salvataggio video terminato.")
        self.video_writer.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        recorder = VideoRecorder()
        rospy.spin()  # Continua a ricevere i messaggi
    except rospy.ROSInterruptException:
        pass
    finally:
        recorder.stop_recording()
