#include "API.h"
#include <Arduino.h>
#include <iostream>

Robot API::robot;
int API::currentX = 0;
int API::currentY = 0;
Direction API::currentDirection = NORTH;

void API::begin() {
  robot.begin();
  Serial.begin(115200);
  delay(2000);
}

int API::mazeWidth() {
  return MAZE_WIDTH;
}

int API::mazeHeight() {
  return MAZE_LENGTH;
}

bool API::wallFront() {
  return robot.isWallFront();
}

bool API::wallRight() {
  return robot.isWallRight();
}

bool API::wallLeft() {
  return robot.isWallLeft();
}

void API::moveForward(int distance) {
  for (int i = 0; i < distance; i++) {
    robot.move(1);

    switch (currentDirection) {
      case NORTH:
        currentY++;
        break;
      case EAST:
        currentX++;
        break;
      case SOUTH:
        currentY--;
        break;
      case WEST:
        currentX--;
        break;
    }

    delay(100);
  }
}

void API::turnRight() {
  robot.turn(90);

  currentDirection = static_cast<Direction>((currentDirection + 1) % 4);

  delay(100);
}

void API::turnLeft() {
  robot.turn(-90);

  currentDirection = static_cast<Direction>((currentDirection + 3) % 4);

  delay(100);
}

void API::setWall(int x, int y, char direction) {
  Serial.print("Set Wall at (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(") direction: ");
  Serial.println(direction);
}

void API::clearWall(int x, int y, char direction) {
  Serial.print("Clear Wall at (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(") direction: ");
  Serial.println(direction);
}

void API::setColor(int x, int y, char color) {
  Serial.print("Set Color at (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(") color: ");
  Serial.println(color);
}

void API::clearColor(int x, int y) {
  Serial.print("Clear Color at (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.println(")");
}

void API::clearAllColor() {
  Serial.println("Clear All Color");
}

void API::setText(int x, int y, const std::string& text) {
  Serial.print("Set Text at (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print("): ");
  Serial.println(text.c_str());
}

void API::clearText(int x, int y) {
  Serial.print("Clear Text at (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.println(")");
}

void API::clearAllText() {
  Serial.println("Clear All Text");
}

bool API::wasReset() {
  return false;
}

void API::ackReset() {
  Serial.println("Acknowledge Reset");
}

void API::printSensors() {
  robot.print_all_sensors();
}