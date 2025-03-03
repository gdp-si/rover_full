"""Display module for the roast application."""
import os
import threading
import time

import cv2
import pygame

from roast.glogging import Logger
from roast.settings import ROOT_DIR

DISPLAY_MODES = {
    "BLINK": cv2.VideoCapture(os.path.join(ROOT_DIR, "data/assets/blink.mp4")),
    "BLINK_THRICE": cv2.VideoCapture(os.path.join(ROOT_DIR, "data/assets/blink_3.mp4")),
    "PATROL": cv2.VideoCapture(os.path.join(ROOT_DIR, "data/assets/patrol.mp4")),
    "BOOTUP": cv2.VideoCapture(os.path.join(ROOT_DIR, "data/assets/bootup.mp4")),
    "THREAT_TRACKING_ON": cv2.VideoCapture(
        os.path.join(ROOT_DIR, "data/assets/threat_tracking_on.mp4")
    ),
    "THREAT_TRACKING_OFF": cv2.VideoCapture(
        os.path.join(ROOT_DIR, "data/assets/threat_tracking_off.mp4")
    ),
}


class Display:
    """Display module for the roast application."""

    LOG = Logger("Display")

    def __init__(self):
        self._window_name = "Roast"
        self._modes = DISPLAY_MODES
        self._video_surface = None
        self._break_thread = False

        pygame.init()
        pygame.display.init()
        pygame.display.set_caption(self._window_name)
        pygame.mouse.set_visible(False)

        self._screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        self._thread = None

    def display(self, mode: str):
        if mode not in self._modes:
            print(f"Mode {mode} not found.")
            return False
        elif mode == "CUSTOM":
            cv2.namedWindow(self._window_name, cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty(
                self._window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
            )
            # TODO: Display latest frame from camera for certain amount of time
        else:
            video = DISPLAY_MODES[mode]
            if self._thread is not None:
                self._break_thread = True
                self._thread.join()
                time.sleep(0.5)
                self._break_thread = False

            self._thread = threading.Thread(
                target=self._play_video, args=(video,), daemon=True
            )
            self._thread.start()
            return True

    def _play_video(self, cap):
        while True:
            ret, frame = cap.read()
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue

            # Calculate aspect ratios
            video_width, video_height = cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(
                cv2.CAP_PROP_FRAME_HEIGHT
            )
            window_width, window_height = (
                self._screen.get_width(),
                self._screen.get_height(),
            )
            video_aspect_ratio = video_width / video_height
            window_aspect_ratio = window_width / window_height

            # Scale video frame to fit window while maintaining aspect ratio
            if video_aspect_ratio > window_aspect_ratio:
                scaled_width = int(window_height * video_aspect_ratio)
                scaled_height = window_height
            else:
                scaled_width = window_width
                scaled_height = int(window_width / video_aspect_ratio)
            frame = cv2.resize(frame, (scaled_width, scaled_height))

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = cv2.rotate(
                frame, cv2.ROTATE_90_CLOCKWISE
            )  # Adjust orientation if needed

            frame = pygame.surfarray.make_surface(frame)
            self._screen.blit(
                frame,
                (
                    (window_width - scaled_width) // 2,
                    (window_height - scaled_height) // 2,
                ),
            )
            pygame.display.flip()

            # Allow delay to be realisitic
            pygame.time.Clock().tick(100)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            if self._break_thread:
                break

    def __del__(self):
        pygame.quit()
