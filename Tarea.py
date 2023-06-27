import math

import cv2
import mediapipe as mp

video = cv2.VideoCapture('video.mp4')
pose = mp.solutions.pose
Pose = pose.Pose(min_tracking_confidence=0.5, min_detection_confidence=0.5)
draw = mp.solutions.drawing_utils


contador = 0
check = True

while True:
    success, img = video.read()
    if not success:
        break
    # videoRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = Pose.process(img)
    points = results.pose_landmarks
    draw.draw_landmarks(img, points, pose.POSE_CONNECTIONS)
    # https://developers.google.com/mediapipe/solutions/vision/pose_landmarker

    h, w, _ = img.shape

    #Catus
    if points:
        left_wrist = points.landmark[15]  # muñeca izquierda
        right_wrist = points.landmark[16]  # muñeca derecha
        left_shoulder = points.landmark[11]  # hombro izquierdo
        right_shoulder = points.landmark[12]  # hombro derecho
        left_elbow = points.landmark[13] # codo izq
        right_elbow = points.landmark[14] #code der
        left_eye = points.landmark[1] #ojo izq
        right_eye = points.landmark[2] #ojo der
        nose = points.landmark[0] #nariz

        # print(abs(left_wrist.y-nose.y)*img.shape[0],'y')
        # print(abs(left_wrist.x - nose.x) * img.shape[1], 'x')


        #Arbol
        if abs(left_wrist.y-nose.y)*img.shape[0] > 85 and abs(left_wrist.x - nose.x) * img.shape[1]<40:
            print("Arbol")

            texto = f'Arbol'
            cv2.rectangle(img, (20, 240), (340, 120), (255, 0, 0), -1)
            cv2.putText(img, texto, (40, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 5)

        # Cactus

        alt_codo_hom_left= abs(left_elbow.y-left_shoulder.y)*img.shape[0]
        alt_codo_hom_right = abs(right_elbow.y - right_shoulder.y) * img.shape[0]
        ancho_codo_muneca_left = abs(left_wrist.x-left_elbow.x)*img.shape[1]
        ancho_codo_muneca_right = abs(right_wrist.x - right_elbow.x) * img.shape[1]
        alto_codo_muneca_right = (right_wrist.y - right_elbow.y) * img.shape[0]*-1
        alto_codo_muneca_left = (left_wrist.y - left_elbow.y) * img.shape[0]*-1

        if alt_codo_hom_left<30 and alt_codo_hom_right<30 and ancho_codo_muneca_left<15 and ancho_codo_muneca_right<15 and alto_codo_muneca_right>0:
            print("Actus")
            texto = f'Actus'
            cv2.rectangle(img, (20, 240), (340, 120), (255, 0, 0), -1)
            cv2.putText(img, texto, (40, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 5)


        if alt_codo_hom_left<30 and ancho_codo_muneca_left<15 and ancho_codo_muneca_right<25 and alto_codo_muneca_right<0:
            print("Actus con una mano abajo")
            texto = f'Actus con una mano abajo'
            cv2.rectangle(img, (20, 240), (340, 120), (255, 0, 0), -1)
            cv2.putText(img, texto, (40, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 5)

        # print(abs(left_wrist.y-nose.y)*img.shape[0],'aa')
        if abs(left_wrist.y-nose.y)*img.shape[0] < 85 and abs(left_wrist.x - nose.x) * img.shape[1]<40:
            print("Corazon")
            texto = f'Corazon'
            cv2.rectangle(img, (20, 240), (340, 120), (255, 0, 0), -1)
            cv2.putText(img, texto, (40, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 5)

        if alto_codo_muneca_right<-40  and alto_codo_muneca_left<-40:
            print("Actus con dos manos abajo, finaliza")
            texto = f'End'
            cv2.rectangle(img, (20, 240), (340, 120), (255, 0, 0), -1)
            cv2.putText(img, texto, (40, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 5)
            break #Termina

    cv2.imshow('Resultado', img)
    cv2.waitKey(40)

