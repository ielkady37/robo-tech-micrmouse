// Phase 1 unit test — mode switch + status LED, see ../HARDWARE_TEST_FLOW.md
// section 1.6.
//
// No driver class exists for these in the main firmware (micromouse.ino
// reads the switch directly; the LED pin only appears in commented-out
// legacy code), so this sketch is standalone -- nothing to symlink.
//
// ---------------------------------------------------------------------------
// WIRING — connect before powering on. Board: ESP32 Dev Module.
// ---------------------------------------------------------------------------
//   ESP32 GPIO 23 -> one leg of the mode switch, other leg to GND.
//     Configured INPUT_PULLUP, so with nothing connected (or the switch
//     open) it reads HIGH; closing the switch to GND reads LOW.
//     NOTE (RD-19, see docs/rd-items/reliability/rd-19-mode-switch-polarity.md):
//     the real firmware's micromouse.ino loads the saved maze when this
//     pin reads HIGH -- i.e. switch OPEN/not pressed, which is the
//     opposite of the naive "press to load" assumption. This test just
//     reports the raw state so you can decide/confirm your wiring
//     convention; it does not replicate that (likely-inverted) behavior.
//   ESP32 GPIO 2  -> status LED anode (through a resistor) -> GND.
// ---------------------------------------------------------------------------
//
// At boot the LED blinks 3 times fast as a self-test independent of the
// switch. After that, the LED simply mirrors the switch: LED ON whenever
// the pin reads LOW (switch closed to GND), OFF when HIGH (open).

#define SWITCH_PIN 23
#define LED_PIN 2

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  Serial.println();
  Serial.println(F("=== Mode switch + LED unit test ==="));
  Serial.println(F("Wiring expected: switch on GPIO23 (to GND), LED on GPIO2"));
  Serial.println(F("LED self-test: 3 fast blinks..."));

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }

  Serial.println(F("Self-test done. LED now mirrors the switch (ON = pin reads LOW)."));
  Serial.println(F("rawState\tinterpreted"));
}

void loop() {
  int raw = digitalRead(SWITCH_PIN);
  bool closedToGround = (raw == LOW);

  digitalWrite(LED_PIN, closedToGround ? HIGH : LOW);

  Serial.print(raw == HIGH ? F("HIGH") : F("LOW"));
  Serial.print(F("\t\t"));
  Serial.println(closedToGround ? F("switch CLOSED (to GND)") : F("switch OPEN"));

  delay(300);
}
