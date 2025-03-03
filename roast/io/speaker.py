"""Speaker class for handling audio output

Audio files are generated using TTSMAKER: https://ttsmaker.com/
"""
import os
import time

import pygame

from roast.settings import ROOT_DIR


class Speaker:
    """Class for handling audio output"""

    def __init__(self, volume=0.5):
        """Initialize Speaker instance"""
        self.volume = volume

        pygame.mixer.init()

        # Load sounds
        self._sounds = {}
        self._load_sounds()

    def _load_sounds(self):
        """Load sound files from disk"""

        SOUND_PATH = os.path.join(ROOT_DIR, "data", "voices")
        for filename in os.listdir(SOUND_PATH):
            if filename.endswith(".mp3"):
                name = filename[:-4]
                path = os.path.join(SOUND_PATH, filename)
                self._sounds[name] = pygame.mixer.Sound(path)

    def play(self, name):
        """Play a sound"""
        self._sounds[name].set_volume(self.volume)
        self._sounds[name].play()

        time.sleep(self._sounds[name].get_length())

    def set_volume(self, volume):
        """Set the volume"""
        self.volume = volume

    def __del__(self):
        """Clean up Speaker instance"""
        pygame.mixer.quit()
