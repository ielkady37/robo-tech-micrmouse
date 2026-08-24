[Home](../../index.md) › [R&D Items](../../index.md) › [Code Health](index.md) › **RD-32**

# RD-32 — Boot Log Prints a Non-Diagnostic String

**Priority:** P3 (Low) · **Status:** Open · **Area:** Code Health · **Source:** `micromouse.ino:84`

## Problem

The boot path prints:

```cpp
Serial.print("ht2oly ana wana wl7dyd etana");
```

A transliterated personal note with zero diagnostic content. Boot logs are the primary forensic tool when a run misbehaves; every line should say what state the machine entered. This line also lacks `println`, so it concatenates with whatever prints next.

## Proposed Approach

```cpp
Serial.println("Boot: loading saved maze from NVS");
```

…paired with the polarity decision from [RD-19](../reliability/rd-19-mode-switch-polarity.md), and an else-branch logging "fresh exploration".

## Acceptance Criteria

- [ ] All boot-path messages are English, terminated, and state the branch taken.
- [ ] A cold log read explains the boot outcome without source access.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-31 Dead code in sketch](rd-31-dead-code-in-sketch.md) | [Code Health](index.md) | [RD-33 Commented-out legacy implementations](rd-33-commented-legacy-code.md) |
