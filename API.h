#pragma once

#include <string>
#include "Robot.h"

#define MAZE_WIDTH 16 //Should be 16 in real thing
#define MAZE_LENGTH 16 //Should be 16 in real thing


// Add the Direction enum at the top
enum Direction { NORTH,
                 EAST,
                 SOUTH,
                 WEST };

class API {
private:
  static Robot robot;
  static int currentX;
  static int currentY;
  static Direction currentDirection;

public:
  static int mazeWidth();
  static int mazeHeight();

  static bool wallFront();
  static bool wallRight();
  static bool wallLeft();

  static void moveForward(int distance = 1);
  static void turnRight();
  static void turnLeft();

  static void setWall(int x, int y, char direction);
  static void clearWall(int x, int y, char direction);

  static void setColor(int x, int y, char color);
  static void clearColor(int x, int y);
  static void clearAllColor();

  static void setText(int x, int y, const std::string& text);
  static void clearText(int x, int y);
  static void clearAllText();

  static bool wasReset();
  static void ackReset();

  static void begin();

  static void printSensors();
};