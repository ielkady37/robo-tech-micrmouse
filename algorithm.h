// Utility functions
uint8_t getCell(uint8_t row, uint8_t col, API api);
uint8_t getRow(uint8_t cell, API api);
uint8_t getCol(uint8_t cell, API api);

// Pathfinding functions
int heuristic(uint8_t row, uint8_t col, bool returning, API api);
void turn(Direction targetDirection, API api);
bool isWallInDirection(Direction direction, API api);
void moveInDirection(Direction direction, API api);
void updateDistancesAStar(bool returning, API api);
Direction getNextMovement(uint8_t currentRow, uint8_t currentCol, bool returning, API api);
void floodFill(API api);
void loadMatrix();
void eraseMatrix();