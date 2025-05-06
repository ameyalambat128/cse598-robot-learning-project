#include <Servo.h>
#include <FastLED.h>

// ——— Configuration ———
#define NUM_SERVOS 5
const uint8_t servoPins[NUM_SERVOS] = {7, 6, 5, 4, 3};
const uint8_t rgbPin               = 13;

// ——— Globals ———
Servo   servos[NUM_SERVOS];
CRGB    leds[1];

// ——— Helpers ———
void testServo(int idx, int angle) {
  if (idx < 0 || idx >= NUM_SERVOS) return;

  // Apply per-servo constraints and flipping:
  switch (idx) {
    case 0:  // Base: below 90 = Right, above 90 = Left
      angle = constrain(angle, 0, 180);
      servos[idx].write(angle);
      break;

    case 1:  // Shoulder: 0 bend front, 180 bend back
    case 2:  // Elbow:    0 bend front, 180 bend back
      angle = constrain(angle, 0, 180);
      servos[idx].write(angle);
      break;

    case 3:  // Wrist: flipped; 180 bend back, 0 bend front
      angle = constrain(angle, 0, 180);
      servos[idx].write(180 - angle);
      break;

    case 4:  // Clamp: do not go below 80 (fully closed at 80)
      angle = constrain(angle, 80, 180);
      servos[idx].write(angle);
      break;
  }

  Serial.print("→ Servo "); Serial.print(idx);
  Serial.print(" = "); Serial.println(angle);
}

void testAllServos() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    testServo(i, 90);  // neutral 90°
    delay(300);
  }
}

void testLED() {
  CRGB colors[] = { CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::White };
  for (auto &c : colors) {
    leds[0] = c;
    FastLED.show();
    Serial.print("→ LED = "); Serial.print(c.r);
    Serial.print(","); Serial.print(c.g);
    Serial.print(","); Serial.println(c.b);
    delay(500);
  }
}

// ——— Arduino Lifecycle ———
void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  Serial.println(F("=== Test Harness Ready ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  S<idx>,<angle>  → move one servo (0-4)"));
  Serial.println(F("  ALL             → reset all to 90°"));
  Serial.println(F("  LED             → cycle RGB LED"));
  Serial.println();

  // Attach servos and set to neutral
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    testServo(i, 90);
  }

  // Initialize LED
  FastLED.addLeds<WS2812, rgbPin, GRB>(leds, 1);
  leds[0] = CRGB::White;
  FastLED.show();
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("S")) {
    int comma = cmd.indexOf(',');
    int idx   = cmd.substring(1, comma).toInt();
    int ang   = cmd.substring(comma + 1).toInt();
    testServo(idx, ang);
  }
  else if (cmd == "ALL") {
    testAllServos();
  }
  else if (cmd == "LED") {
    testLED();
  }
  else {
    Serial.print(F("Unknown cmd: ")); Serial.println(cmd);
  }
}
