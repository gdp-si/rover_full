// Test code for weapon module
// Single Axis 4A TB6600 Stepper Motor Driver Controller
// use Serial Monitor to control With115200 baud
#include <FastLED.h>
#include <Servo.h>
Servo myservo;
int pos = 0;
#define NUM_LEDS 100
#define DATA_PIN 6
#define COLOUR_ORDER RGB
#define CHIPSET WS2812B
#define BRIGHTNESS 255
// #define VOLTS 5
// #define MAX_AMPS 500
String inputString = "";
boolean stringComplete = true;
CRGB leds[NUM_LEDS];

void setup() {
  Serial.begin(115200);
  myservo.attach(9);
  Triggeroff();
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
}
void serialEvent()  // ********************************************************
                    // Serial in
{
  while (Serial.available()) {
    char inChar = (char)Serial.read();  // get the new byte:
    if (inChar > 0) {
      inputString += inChar;  // add it to the inputString:
    }
    if (inChar == '\n') {
      stringComplete =
          true;  // if the incoming character is a newline, set a
                 // flag so the main loop can do something about it:
    }
    Serial.println(inChar);
  }
}

//// code for weapon module

void Triggeron() { myservo.write(0); }

void Triggeroff() { myservo.write(90); }

/// code for led strips

void blue_on() {
  Serial.begin(115200);
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = CRGB ::Blue;
    FastLED.show();
    delayMicroseconds(0.1);
  }
}
void red_on() {
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = CRGB ::Red;
    FastLED.show();
    delayMicroseconds(0.1);
  }
}

void leds_off() {
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = CRGB ::Black;
    FastLED.show();
    delayMicroseconds(0.1);
  }
}

void pattern_1() {
  for (int k = 0; k <= 2; k++) {
    for (int i = 0; i <= 4; i++) {
      leds[0] = CRGB(255, 0, 0);
      leds[1] = CRGB(255, 0, 0);
      leds[2] = CRGB(255, 0, 0);
      leds[3] = CRGB(255, 0, 0);
      leds[4] = CRGB(255, 0, 0);
      leds[5] = CRGB(255, 0, 0);
      leds[6] = CRGB(255, 0, 0);
      leds[7] = CRGB(255, 0, 0);
      leds[8] = CRGB(255, 0, 0);
      leds[9] = CRGB(255, 0, 0);

      leds[10] = CRGB(0, 0, 255);
      leds[11] = CRGB(0, 0, 255);
      leds[12] = CRGB(0, 0, 255);
      leds[13] = CRGB(0, 0, 255);
      leds[14] = CRGB(0, 0, 255);
      leds[15] = CRGB(0, 0, 255);
      leds[16] = CRGB(0, 0, 255);
      leds[17] = CRGB(0, 0, 255);
      leds[18] = CRGB(0, 0, 255);
      leds[19] = CRGB(0, 0, 255);

      FastLED.show();
      delay(10);
      FastLED.clear();
      delay(10);
    }

    for (int i = 0; i <= 4; i++) {
      leds[0] = CRGB(0, 0, 255);
      leds[1] = CRGB(0, 0, 255);
      leds[2] = CRGB(0, 0, 255);
      leds[3] = CRGB(0, 0, 255);
      leds[4] = CRGB(0, 0, 255);
      leds[5] = CRGB(0, 0, 255);
      leds[6] = CRGB(0, 0, 255);
      leds[7] = CRGB(0, 0, 255);
      leds[8] = CRGB(0, 0, 255);
      leds[9] = CRGB(0, 0, 255);

      leds[10] = CRGB(255, 0, 0);
      leds[11] = CRGB(255, 0, 0);
      leds[12] = CRGB(255, 0, 0);
      leds[13] = CRGB(255, 0, 0);
      leds[14] = CRGB(255, 0, 0);
      leds[15] = CRGB(255, 0, 0);
      leds[16] = CRGB(255, 0, 0);
      leds[17] = CRGB(255, 0, 0);
      leds[18] = CRGB(255, 0, 0);
      leds[19] = CRGB(255, 0, 0);
      FastLED.show();
      delay(10);
      FastLED.clear();
      delay(10);
    }
  }
}

void loop() {
  serialEvent();
  {
    if (stringComplete) {
      /// weapon module

      if (inputString == "triggeron\n") {
        Triggeron();
      } else if (inputString == "triggerof\n") {
        Triggeroff();
      }

      /// led strips

      else if (inputString == "red\n") {
        red_on();
      }
      if (inputString == "blue\n") {
        blue_on();
      } else if (inputString == "blink\n") {
        pattern_1();
      } else if (inputString == "off\n") {
        leds_off();
      }

      if (inputString != "") {
        Serial.println("entered command is =" + inputString);
      }
      inputString = "";
      stringComplete = false;  // clear the string:
    }
  }
}
