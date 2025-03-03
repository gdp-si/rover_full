import os

import cv2
import mediapipe as mp
import numpy as np
from tensorflow.keras.models import load_model

from roast.settings import ROOT_DIR


class HandGestureRecognition:
    """Hand Gesture Recognition"""

    def __init__(self, debug: bool = False):
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mpDraw = mp.solutions.drawing_utils
        self._model_path = os.path.join(ROOT_DIR, "models/hand_gesture_recog")
        self.model = load_model(self._model_path)

        self._debug = debug

        with open(os.path.join(self._model_path, "gesture.names"), "r") as f:
            self.classNames = f.read().split("\n")

    def recognize_gesture(self, frame):
        """Recognize gesture from frame"""

        frame = self._preprocess_frame(frame)
        landmarks = self._detect_landmarks(frame)

        if len(landmarks) == 0:
            return {
                "is_gesture": False,
                "gesture": "",
                "num_gestures_detected": 0,
            }

        # Get prediction
        prediction = self.model.predict([landmarks])
        classID = np.argmax(prediction)
        className = self.classNames[classID]

        if self._debug:
            cv2.putText(
                frame,
                className,
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("Output", frame)

        return {
            "is_gesture": bool(len(prediction) > 0),
            "gesture": className,
            "num_gestures_detected": len(prediction),
        }

    def _preprocess_frame(self, frame):
        frame = cv2.flip(frame, 1)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        return frame

    def _detect_landmarks(self, frame):
        x, y, _ = frame.shape
        result = self.hands.process(frame)
        landmarks = []

        if result.multi_hand_landmarks:
            for handslms in result.multi_hand_landmarks:
                for lm in handslms.landmark:
                    lmx = int(lm.x * x)
                    lmy = int(lm.y * y)
                    landmarks.append([lmx, lmy])

        return landmarks

    def __del__(self):
        del self.hands
        del self.mpHands
        del self.model


def main():
    """Main function"""
    gesture_recognizer = HandGestureRecognition(debug=True)
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        _, frame = cap.read()
        result = gesture_recognizer.recognize_gesture(frame)
        print(result)
        cv2.imshow("Image", frame)
        cv2.waitKey(1)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    input("Press Enter to continue...")
