#include <cmath>
#include <iostream>
#include <queue>
#include <vector>
#include <functional>
#include <climits>
#include "API.h"
#include <cstdint>
#include "algorithm.h"
#include <Preferences.h>

Preferences prefs;

// Cell representation utilities
uint8_t getCell(uint8_t row, uint8_t col, API api) {
  return (col << 4) | (row & 0x0F);
}

uint8_t getRow(uint8_t cell, API api) {
  return cell & 0x0F;
}

uint8_t getCol(uint8_t cell, API api) {
  return (cell >> 4) & 0x0F;
}

// Global variables
Direction currentDirection = NORTH;
uint8_t distance[MAZE_LENGTH][MAZE_WIDTH] = { 0 };

//Save these --------
bool hasNorthWall[MAZE_LENGTH][MAZE_WIDTH] = { false };
bool hasEastWall[MAZE_LENGTH][MAZE_WIDTH] = { false };


void save() {
  prefs.begin("maze", false);
  prefs.putBytes("north", hasNorthWall, sizeof(hasNorthWall));  // Save whole block
  prefs.putBytes("east", hasEastWall, sizeof(hasEastWall));     // Save whole block
  prefs.putBytes("distance", distance, sizeof(distance));     // Save whole block
  prefs.end();
}
void eraseMatrix() {
  prefs.begin("maze", false);  // open namespace in RW mode
  prefs.clear();                  // erase all keys in this namespace
  prefs.end();                    // close prefs
}
void loadMatrix() {
  prefs.begin("maze", true);
  prefs.getBytes("north", hasNorthWall, sizeof(hasNorthWall));  // Restore whole block
  prefs.getBytes("east", hasEastWall, sizeof(hasEastWall));     // Restore whole block
  prefs.getBytes("distance", distance, sizeof(distance));     // Restore whole block
  prefs.end();

  Serial.println("North Wall: ");
  for (int i = MAZE_LENGTH -1 ; i >= 0 ; i--) {
    for (int j = 0; j < MAZE_WIDTH; j++) {
      Serial.print(hasNorthWall[i][j]);  // flatten index
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println("--------");
  
  Serial.println("East Wall: ");
  for (int i = MAZE_LENGTH - 1; i >= 0; i--) {
    for (int j = 0; j < MAZE_WIDTH; j++) {
      Serial.print(hasEastWall[i][j]);  // flatten index
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println("--------");
    
  Serial.println("Distance: ");
  for (int i = MAZE_LENGTH - 1; i >= 0; i--) {
    for (int j = 0; j < MAZE_WIDTH; j++) {
      Serial.print(distance[i][j]);  // flatten index
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println("--------");
  
}


// --------------

uint8_t startingRow = 0;
uint8_t startingCol = 0;  // Should be 0 in real thing

//Enter lower bounds of goal square
uint8_t targetRow = 7;  // Should be 7 in real thing
uint8_t targetCol = 7;  // Should be 7 in real thing

uint8_t currentRow = 0;
uint8_t currentCol = 0;

// A* Node structure
struct Node {
  uint8_t row;
  uint8_t col;
  int f;  // f = g + h
  int g;  // cost from start

  Node(uint8_t r, uint8_t c, int g_val, int h_val)
    : row(r), col(c), g(g_val), f(g_val + h_val) {}

  bool operator>(const Node& other) const {
    return f > other.f;  // For min-heap
  }
};

// Heuristic function (Manhattan distance)
int heuristic(uint8_t row, uint8_t col, bool returning, API api) {
  if (returning) {
    // Distance to start (0,0)
    return row + col;
  } else {
    // Distance to closest center cell
    int centerRow = (row < targetRow + 1) ? targetRow : targetRow + 1;
    int centerCol = (col < targetCol + 1) ? targetCol : targetCol + 1;
    return abs((int)row - centerRow) + abs((int)col - centerCol);
  }
}

// void log(const std::string& text) {
//   std::cerr << text << std::endl;
// }

void turn(Direction targetDirection, API api) {
  int leftTurns = (currentDirection - targetDirection + 4) % 4;
  int rightTurns = (targetDirection - currentDirection + 4) % 4;

  if (leftTurns <= rightTurns) {
    for (int i = 0; i < leftTurns; i++) {
      api.turnLeft();
    }
  } else {
    for (int i = 0; i < rightTurns; i++) {
      api.turnRight();
    }
  }

  currentDirection = targetDirection;
}

bool isWallInDirection(Direction direction, API api) {
  if (direction == currentDirection) return api.wallFront();
  if (direction == static_cast<Direction>((currentDirection + 3) % 4)) return api.wallLeft();
  if (direction == static_cast<Direction>((currentDirection + 1) % 4)) return api.wallRight();
  return false;  // Should never reach here
}

void moveInDirection(Direction direction, API api) {
  turn(direction, api);
  api.moveForward(1);
}

void updateDistancesAStar(bool returning, API api) {
  // Reset distances to "infinity"
  for (int i = 0; i < MAZE_LENGTH; i++) {
    for (int j = 0; j < MAZE_WIDTH; j++) {
      distance[i][j] = UINT8_MAX;
    }
  }

  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;

  // Initialize start/goal nodes
  if (returning) {
    distance[0][0] = 0;
    openSet.push(Node(0, 0, 0, heuristic(0, 0, returning, api)));
  } else {

    distance[targetRow][targetCol] = 0;
    distance[targetRow][targetCol + 1] = 0;
    distance[targetRow + 1][targetCol] = 0;
    distance[targetRow + 1][targetCol + 1] = 0;

    openSet.push(Node(targetRow, targetCol, 0, heuristic(targetRow, targetCol, returning, api)));
    openSet.push(Node(targetRow, targetCol + 1, 0, heuristic(targetRow, targetCol + 1, returning, api)));
    openSet.push(Node(targetRow + 1, targetCol, 0, heuristic(targetRow + 1, targetCol, returning, api)));
    openSet.push(Node(targetRow + 1, targetCol + 1, 0, heuristic(targetRow + 1, targetCol + 1, returning, api)));
  }

  while (!openSet.empty()) {
    Node current = openSet.top();
    openSet.pop();

    uint8_t row = current.row;
    uint8_t col = current.col;

    // Check if we found a better path to this node
    if (current.g > distance[row][col]) {
      continue;
    }

    // Check all four directions for neighbors
    // North
    if (row < MAZE_LENGTH - 1 && !hasNorthWall[row][col]) {
      uint8_t newRow = row + 1;
      uint8_t newCol = col;
      int newG = distance[row][col] + 1;

      if (newG < distance[newRow][newCol]) {
        distance[newRow][newCol] = newG;
        int h = heuristic(newRow, newCol, returning, api);
        openSet.push(Node(newRow, newCol, newG, h));
      }
    }

    // South
    if (row > 0 && !hasNorthWall[row - 1][col]) {
      uint8_t newRow = row - 1;
      uint8_t newCol = col;
      int newG = distance[row][col] + 1;

      if (newG < distance[newRow][newCol]) {
        distance[newRow][newCol] = newG;
        int h = heuristic(newRow, newCol, returning, api);
        openSet.push(Node(newRow, newCol, newG, h));
      }
    }

    // East
    if (col < MAZE_WIDTH - 1 && !hasEastWall[row][col]) {
      uint8_t newRow = row;
      uint8_t newCol = col + 1;
      int newG = distance[row][col] + 1;

      if (newG < distance[newRow][newCol]) {
        distance[newRow][newCol] = newG;
        int h = heuristic(newRow, newCol, returning, api);
        openSet.push(Node(newRow, newCol, newG, h));
      }
    }

    // West
    if (col > 0 && !hasEastWall[row][col - 1]) {
      uint8_t newRow = row;
      uint8_t newCol = col - 1;
      int newG = distance[row][col] + 1;

      if (newG < distance[newRow][newCol]) {
        distance[newRow][newCol] = newG;
        int h = heuristic(newRow, newCol, returning, api);
        openSet.push(Node(newRow, newCol, newG, h));
      }
    }
  }
}

Direction getNextMovement(uint8_t currentRow, uint8_t currentCol, bool returning, API api) {
  uint8_t minDistance = distance[currentRow][currentCol];
  Direction bestDirection = NORTH;

  // Check all possible moves
  if (currentRow < MAZE_LENGTH - 1 && !hasNorthWall[currentRow][currentCol] && distance[currentRow + 1][currentCol] <= minDistance) {
    minDistance = distance[currentRow + 1][currentCol];
    bestDirection = NORTH;
  }

  if (currentRow > 0 && !hasNorthWall[currentRow - 1][currentCol] && distance[currentRow - 1][currentCol] <= minDistance) {
    minDistance = distance[currentRow - 1][currentCol];
    bestDirection = SOUTH;
  }

  if (currentCol < MAZE_WIDTH - 1 && !hasEastWall[currentRow][currentCol] && distance[currentRow][currentCol + 1] <= minDistance) {
    minDistance = distance[currentRow][currentCol + 1];
    bestDirection = EAST;
  }

  if (currentCol > 0 && !hasEastWall[currentRow][currentCol - 1] && distance[currentRow][currentCol - 1] <= minDistance) {
    minDistance = distance[currentRow][currentCol - 1];
    bestDirection = WEST;
  }

  return bestDirection;
}

// void logDirection(Direction d) {
//   if (d == NORTH) log("North");
//   else if (d == EAST) log("East");
//   else if (d == SOUTH) log("South");
//   else log("West");
// }

void floodFill(API api) {
  bool returning = false;
  updateDistancesAStar(returning, api);

  while (distance[currentRow][currentCol] != 0) {
    Serial.print("Current Cell: ");
    Serial.print(currentRow);
    Serial.print(", ");
    Serial.println(currentCol);

    // Update wall information
    if (isWallInDirection(NORTH, api)) {
      hasNorthWall[currentRow][currentCol] = true;
      Serial.println("Detected North Wall!");
    }
    if (isWallInDirection(EAST, api)) {
      hasEastWall[currentRow][currentCol] = true;
      Serial.println("Detected East Wall! ");
    }
    if (currentRow > 0 && isWallInDirection(SOUTH, api)) {
      hasNorthWall[currentRow - 1][currentCol] = true;
      Serial.println("Detected South Wall! ");
    }
    if (currentCol > 0 && isWallInDirection(WEST, api)) {
      hasEastWall[currentRow][currentCol - 1] = true;
      Serial.println("Detected West Wall! ");
    }

    // Recalculate path if new walls were discovered
    bool wallsDetected = api.wallFront() || api.wallLeft() || api.wallRight();
    if (wallsDetected) {
      updateDistancesAStar(returning, api);
    }

    Direction nextDirection = getNextMovement(currentRow, currentCol, returning, api);
    // logDirection(nextDirection);
    Serial.print("Going ");
    if (nextDirection == NORTH) {
      Serial.println("North");
    } else if (nextDirection == EAST) {
      Serial.println("East");
    } else if (nextDirection == SOUTH) {
      Serial.println("South");
    } else {
      Serial.println("West");
    }
    // delay(2000);
    moveInDirection(nextDirection, api);

    // Update position
    if (nextDirection == NORTH) currentRow++;
    else if (nextDirection == EAST) currentCol++;
    else if (nextDirection == SOUTH) currentRow--;
    else if (nextDirection == WEST) currentCol--;

    currentDirection = nextDirection;
    save();
  }
}