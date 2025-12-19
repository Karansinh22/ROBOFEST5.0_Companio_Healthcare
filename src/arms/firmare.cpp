#include <Servo.h>

Servo servos[18];
int angle = 90;

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 18; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(60);   // neutral
  }

  Serial.println("18-Servo Controller Ready");
}

void loop() {
  if (Serial.available()) {
    angle = Serial.parseInt();
    angle = constrain(angle, 0, 180);

    for (int i = 0; i < 18; i++) {
      servos[i].write(angle);
    }

    Serial.print("Angle set to: ");
    Serial.println(angle);
  }
}
