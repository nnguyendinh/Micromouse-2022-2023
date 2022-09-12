#ifndef SOLVER_H
#define SOLVER_H

typedef enum Heading {NORTH, EAST, SOUTH, WEST} Heading;
typedef enum Action {LEFT, FORWARD, RIGHT, IDLE} Action;
typedef enum Algorithm {DEAD, FLOODFILL} Algorithm;

struct Cell
{
    int row;
    int col;
};

struct Cell* newCell(int x_, int y_);  // Initialize an existing cell with values x and y
void insertQueue(struct Cell* input);
void popQueueFront();
struct Cell* queueFront();

//void restart();

Action solver(Algorithm alg);
Action deadReckoning();
Action floodFill();


#endif
