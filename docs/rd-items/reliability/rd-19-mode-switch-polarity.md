[Home](../../index.md) › [R&D Items](../../index.md) › [Reliability](index.md) › **RD-19**

# RD-19 — Mode Switch Polarity Is Likely Inverted

**Priority:** P2 (Medium) · **Status:** Open · **Area:** Reliability · **Source:** `micromouse.ino:78-86`

## Problem

The mode switch uses a pull-up, so an unpressed switch reads HIGH:

```cpp
pinMode(solveSwitch, INPUT_PULLUP);
...
if (digitalRead(solveSwitch)) {        // true when NOT pressed
  Serial.print("ht2oly ana wana wl7dyd etana");
  loadMatrix();
}
```

As written, the saved maze loads when the switch is *open*. The natural intent — "press to load" — is exactly backwards. It may also be intentional ("loads by default; press to explore fresh"), but nothing documents that convention, which makes field behavior ambiguous.

## Proposed Approach

Decide the convention and encode it explicitly:

```cpp
const bool pressed = digitalRead(solveSwitch) == LOW;
if (pressed) {
  Serial.println("Loading saved maze from NVS");
  loadMatrix();
}
```

…or keep default-load behavior but document it at the pin definition. While here, replace the debug string ([RD-32](../code-health/rd-32-debug-string-quality.md)).

## Acceptance Criteria

- [ ] Switch behavior documented next to the pin definition.
- [ ] Bench check: both switch states produce the documented boot behavior.
- [ ] Log message states plainly which branch was taken.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-15 Hang on init failure](rd-15-hang-on-init-failure.md) | [Reliability](index.md) | [Code Health](../code-health/index.md) |
