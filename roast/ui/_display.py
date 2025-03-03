import math
import sys

import pygame


class LCDMatrix:
    """This class is used to control the LCD Matrix."""

    def __init__(self, width, height, screen_callback=None):
        pygame.init()

        self._width = width
        self._height = height
        self._screen = pygame.display.set_mode((width, height))
        self._clock = pygame.time.Clock()
        self._fps = 60  # Set your desired frames per second
        self.screen_callback = screen_callback or self._screen_callback

        pygame.display.set_caption("LCD Matrix")

        # Disable the mouse cursor
        pygame.mouse.set_visible(False)

        # Disable the title bar
        # pygame.display.iconify()

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

    def set_brightness(self, brightness):
        """Set the brightness of the LCD Matrix."""
        # Set the brightness for the window
        self._screen.set_alpha(brightness)

    def clear(self):
        """Clear the LCD Matrix."""
        self._screen.fill((0, 0, 0))

    def run(self):
        """Run the LCD Matrix."""
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            # Run the screen callback
            self.screen_callback(self._screen)

            # Update the screen
            pygame.display.flip()

            self._clock.tick(self._fps)

    def __del__(self):
        pygame.quit()

    def __str__(self):
        return f"LCDMatrix({self._width}, {self._height})"

    def _screen_callback(self, screen):
        """This function get screen_mat and send it."""
        raise NotImplementedError


class EyeWink:
    """EyeWink class."""

    def __init__(self, matrix):
        self.matrix = matrix
        self.eye1_x, self.eye1_y = 200, 300
        self.eye2_x, self.eye2_y = 600, 300

        self._angle = 0

    def screen_callback(self, screen):
        """Screen callback."""
        # Clear the screen
        screen.fill((0, 0, 0))

        # Create surface
        width = screen.get_width()
        height = screen.get_height()
        eyes = pygame.Surface((width, height), pygame.SRCALPHA)

        # Draw the eyeballs
        pygame.draw.circle(eyes, (255, 255, 255), (self.eye1_x, self.eye1_y), 80)
        pygame.draw.circle(eyes, (0, 0, 0), (self.eye1_x, self.eye1_y), 60)
        pygame.draw.circle(eyes, (255, 255, 255), (self.eye2_x, self.eye2_y), 80)
        pygame.draw.circle(eyes, (0, 0, 0), (self.eye2_x, self.eye2_y), 60)

        # Draw the pupils
        eyes_rect = eyes.get_rect(center=screen.get_rect().center)
        new_height = round(eyes_rect.height * math.sin(math.radians(self._angle)))
        self._angle += 2

        wink_eyes = (
            eyes if new_height >= 0 else pygame.transform.flip(eyes, False, True)
        )
        wink_eyes = pygame.transform.scale(
            wink_eyes, (eyes_rect.width, abs(new_height))
        )
        screen.blit(wink_eyes, wink_eyes.get_rect(center=eyes_rect.center))


class HappyEyes:
    """HappyEyes class."""

    def __init__(self, matrix):
        self.matrix = matrix
        self.eye1_x, self.eye1_y = 200, 300
        self.eye2_x, self.eye2_y = 600, 300

        self._scale = 1
        self._invert_behavior = False
        self._reset = False

    @property
    def reset(self):
        return self._reset

    @reset.setter
    def reset(self, value):
        self._reset = value

    def screen_callback(self, screen):
        """Screen callback."""
        # Clear the screen
        screen.fill((0, 0, 0))

        # Create surface
        width = screen.get_width()
        height = screen.get_height()
        eyes = pygame.Surface((width, height), pygame.SRCALPHA)

        # Draw the eyeballs
        pygame.draw.circle(eyes, (255, 255, 255), (self.eye1_x, self.eye1_y), 80)
        pygame.draw.circle(eyes, (0, 0, 0), (self.eye1_x, self.eye1_y), 60)
        pygame.draw.circle(eyes, (255, 255, 255), (self.eye2_x, self.eye2_y), 80)
        pygame.draw.circle(eyes, (0, 0, 0), (self.eye2_x, self.eye2_y), 60)

        # Draw a rectangle that cuts the eyes into half
        pygame.draw.rect(
            eyes, (0, 0, 0), (0, height / (2 - self._scale), width, height)
        )
        if self._scale > 0.1 and not self._invert_behavior:
            self._scale -= 0.01
        # elif self._scale < 0.9 and self._invert_behavior:
        #     self._scale += 0.01
        # else:
        #     self._invert_behavior = not self._invert_behavior
        #     self._scale = 0 if self._invert_behavior else 1

        screen.blit(eyes, (0, 0))


class SleepEyes:
    """SleepEyes class."""

    def __init__(self, matrix):
        self.matrix = matrix
        self.eye1_x, self.eye1_y = 200, 300
        self.eye2_x, self.eye2_y = 600, 300

        self._scale = 0

    def screen_callback(self, screen):
        """Screen callback."""
        # Clear the screen
        screen.fill((0, 0, 0))

        # Create surface
        width = screen.get_width()
        height = screen.get_height()
        eyes = pygame.Surface((width, height), pygame.SRCALPHA)

        # Draw the eyeballs
        pygame.draw.circle(eyes, (255, 255, 255), (self.eye1_x, self.eye1_y), 80)
        pygame.draw.circle(eyes, (0, 0, 0), (self.eye1_x, self.eye1_y), 60)
        pygame.draw.circle(eyes, (255, 255, 255), (self.eye2_x, self.eye2_y), 80)
        pygame.draw.circle(eyes, (0, 0, 0), (self.eye2_x, self.eye2_y), 60)

        # Draw a rectangle that cuts the eyes into half
        pygame.draw.rect(eyes, (0, 0, 0), (0, 0, width, height * self._scale / 1.75))
        if self._scale < 0.9:
            self._scale += 0.01

        screen.blit(eyes, (0, 0))


def main():
    """Main function."""

    matrix = LCDMatrix(800, 600)
    # eyes = EyeWink(matrix)
    # eyes = HappyEyes(matrix)
    eyes = SleepEyes(matrix)
    matrix.screen_callback = eyes.screen_callback

    matrix.run()


if __name__ == "__main__":
    main()
