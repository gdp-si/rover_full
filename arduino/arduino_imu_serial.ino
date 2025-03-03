#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  delay(100);
  Serial.begin(115200);
  while (!Serial) delay(100);  // wait for serial port to open!
  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1)
      ;
  }

  delay(10);
  bno.setExtCrystalUse(true);
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);
  imu::Vector<3> calibratedOrientation =
      bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print("Orientation: ");
  Serial.print(event.orientation.x);
  Serial.print(" ");
  Serial.print(event.orientation.y);
  Serial.print(" ");
  Serial.println(event.orientation.z);

  delay(10);
}
