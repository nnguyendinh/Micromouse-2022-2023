#include "solver.h"
#include "pid.h"

#include <stdlib.h>
#include <stdio.h>

int initialized = 0;
extern int16_t startPressed;
extern int running;

struct Cell* currPos;
Heading currHead;
int Manhattans[16][16];
struct Cell* queue[512];
int queueStart; //assuming circular queue, this helps us keep track of what position we are in in the queue, in terms of the "start"
int queueEnd; //keep track of end of queue, where to add next

int horzWall[17][16]; // got rid of the extra in the array to have hopefully less confusion
int vertWall[16][17];

extern int16_t leftIRvalue;
extern int16_t rightIRvalue;
extern int16_t frontLeftIRvalue;
extern int16_t frontRightIRvalue; // TODO: IS THIS NECESSARY?

struct Cell* newCell(int r, int c)           // Acts as a constructor for a cell cuz C is annoying
{
    struct Cell* p = malloc(sizeof(struct Cell));
    p->row = r;
    p->col = c;
}

void insertQueue(struct Cell* input) {
    queue[queueEnd] = input;

    queueEnd++;

    if (queueEnd == 512) {
        queueEnd = 0;
        //reset cause circular queue
    }
    //check me on this i might've messed up on pointers, i'm doing this right off of github and not from a compiler lol
}

void popQueueFront()
{
    free(queueFront());

    queueStart++;
    if (queueStart == 512) {
        queueStart = 0;
        //reset cause circular queue
    }
}

struct Cell* queueFront()
{
    return queue[queueStart];
}

void initElements()
{
    currPos = newCell(15, 0);           // Sets current position to row 15, column 0
    currHead = NORTH;                    // Sets current heading to north
    for (int j = 0; j < 8; j++)                // Initializes default Manhattan distances for empty maze
    {
        for (int i = 0; i < 8; i++)
        {
            Manhattans[i][j] = 14 - i - j;
            Manhattans[15 - i][j] = 14 - i - j;
            Manhattans[i][15 - j] = 14 - i - j;
            Manhattans[15 - i][15 - j] = 14 - i - j;
        }
    }

    for (int i = 0; i < 17; i++) {
        for (int j = 0; j < 16; j++) {
            horzWall[i][j] = 0;
        }
    }

    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 17; j++) {
            vertWall[i][j] = 0;
        }
    }

    queueStart = 0;
    queueEnd = 0;
}

void restart() {

	resetPID();
	currPos = newCell(15, 0);           // Sets current position to row 15, column 0
	currHead = NORTH;                    // Sets current heading to north
	queueStart = 0;
	queueEnd = 0;
	startPressed = 0;
	running = 0;

}

void displayManhatttans()       // Displays all current manhattan distances in grid
{
//    for (int row = 0; row < 16; row++)
//        for (int col = 0; col < 16; col++)
//        {
//            char str[4];
//            sprintf(str, "%d", Manhattans[row][col]);
//            API_setText(col, 15 - row, str);
//        }
}

void setWall(int dir)
{
    int currX = currPos->col;
    int currY = 15 - currPos->row;

    switch (dir)
    {
    case NORTH:
        horzWall[currPos->row][currPos->col] = 1;   // Sets the 2D array value to 1 to represent true (there's no bool type in C)
        //API_setWall(currX, currY, 'n');             // Light up the discovered wall in the simulator
        break;
    case EAST:
        vertWall[currPos->row][currPos->col + 1] = 1;   // May need to check 2D array logic my head hurts lol
        //API_setWall(currX, currY, 'e');
        break;
    case SOUTH:
        horzWall[currPos->row + 1][currPos->col] = 1;
        //API_setWall(currX, currY, 's');
        break;
    case WEST:
        vertWall[currPos->row][currPos->col] = 1;
        //API_setWall(currX, currY, 'w');
        break;
    }
}

void detectWalls()
{
    switch (currHead)
    {
    case NORTH:
        if (frontWallCheck())
        {
            setWall(NORTH);
        }
        if (leftWallCheck())
        {
            setWall(WEST);
        }
        if (rightWallCheck())
        {
            setWall(EAST);
        }
        break;
    case EAST:
        if (frontWallCheck())
        {
            setWall(EAST);
        }
        if (leftWallCheck())
        {
            setWall(NORTH);
        }
        if (rightWallCheck())
        {
            setWall(SOUTH);
        }
        break;
    case SOUTH:
        if (frontWallCheck())
        {
            setWall(SOUTH);
        }
        if (leftWallCheck())
        {
            setWall(EAST);
        }
        if (rightWallCheck())
        {
            setWall(WEST);
        }
        break;
    case WEST:
        if (frontWallCheck())
        {
            setWall(WEST);
        }
        if (leftWallCHeck())
        {
            setWall(SOUTH);
        }
        if (rightWallCheck())
        {
            setWall(NORTH);
        }
        break;
    }
}

void recalculate()
{

    insertQueue(newCell(currPos->row, currPos->col));

    //while queue is not empty
    while (queueStart != queueEnd) {

        //Take front cell in queue “out of line” for consideration

        struct Cell* currElement = queueFront(); //has the current compared

//        char str[50];
//        sprintf(str, "Calculating distances at row %d, column %d", currElement->row, currElement->col);
//        debug_log(str);

        //Get the front cell’s minimum value amongst accessible neighbors.

        int neighborMinimum = -1;       //uninitialized or uncompared state when less than 0
        if (horzWall[currElement->row][currElement->col] != 1) {        //north wall
            if (neighborMinimum < 0 || neighborMinimum > Manhattans[currElement->row - 1][currElement->col]) {
                neighborMinimum = Manhattans[currElement->row - 1][currElement->col];
            }
        }
        if (vertWall[currElement->row][currElement->col + 1] != 1) {        //east wall
            if (neighborMinimum < 0 || neighborMinimum > Manhattans[currElement->row][currElement->col + 1]) {
                neighborMinimum = Manhattans[currElement->row][currElement->col + 1];
            }
        }
        if (horzWall[currElement->row + 1][currElement->col] != 1) {        //south wall
            if (neighborMinimum < 0 || neighborMinimum > Manhattans[currElement->row + 1][currElement->col]) {
                neighborMinimum = Manhattans[currElement->row + 1][currElement->col];
            }
        }
        if (vertWall[currElement->row][currElement->col] != 1) {        //west wall
            if (neighborMinimum < 0 || neighborMinimum > Manhattans[currElement->row][currElement->col - 1]) {
                neighborMinimum = Manhattans[currElement->row][currElement->col - 1];
            }
        }
        //If the front cell’s value ≤ minimum of its neighbors,
        //set the front cell’s value to minimum + 1 and add all accessible neighbors to the queue.
        if (Manhattans[currElement->row][currElement->col] <= neighborMinimum) {
            Manhattans[currElement->row][currElement->col] = neighborMinimum + 1;

            //again set through the accessible ones and add to queue
            if (horzWall[currElement->row][currElement->col] != 1) {        //north wall
                insertQueue(newCell(currElement->row - 1, currElement->col));
                //char str[50];
                //sprintf(str, "Added row %d, column %d to the queue", currElement->row - 1, currElement->col);
                //debug_log(str);
            }
            if (vertWall[currElement->row][currElement->col + 1] != 1) {        //east wall
                insertQueue(newCell(currElement->row, currElement->col + 1));
                //char str[50];
                //sprintf(str, "Added row %d, column %d to the queue", currElement->row, currElement->col + 1);
                //debug_log(str);
            }
            if (horzWall[currElement->row + 1][currElement->col] != 1) {        //south wall
                insertQueue(newCell(currElement->row + 1, currElement->col));
                //char str[50];
                //sprintf(str, "Added row %d, column %d to the queue", currElement->row + 1, currElement->col);
                //debug_log(str);
            }
            if (vertWall[currElement->row][currElement->col] != 1) {        //west wall
                insertQueue(newCell(currElement->row, currElement->col - 1));
                //char str[50];
                //sprintf(str, "Added row %d, column %d to the queue", currElement->row, currElement->col - 1);
                //debug_log(str);
            }
            //we might have to check edge conditions (i.e. checking that we don't access -1 rows or something)
        }

        popQueueFront();      // Deletes cell from queue and frees memory

        //Else, continue!
    }
}

Action solver(Algorithm alg) {
    switch (alg)
    {
    case DEAD:
    	return deadReckoning();
    	break;
    case FLOODFILL:
    	return floodFill();
    	break;
    }
}

Action deadReckoning() {       // The simple left wall following algorithm that they provided
	if (!frontWallCheck())
	{
		return FORWARD;
	}
	else if (!rightWallCheck())
	{
		return RIGHT;
	}
	else
	{
		return LEFT;
	}
}

Action floodFill() {
    if (!initialized)           // Initializes all the elements once (there might be a better way to do this idk)
    {
        initElements();
        initialized = 1;
    }

    detectWalls();  // Lights up detected walls and adds them to the 2D wall arrays
    displayManhatttans();

    int nextHead = -1;
    int row = currPos->row;
    int col = currPos->col;

    if (Manhattans[row][col] == 0)
    {
		restart();
    	return IDLE;
    }


    int northBlocked = horzWall[currPos->row][currPos->col];
    int eastBlocked = vertWall[currPos->row][currPos->col + 1];
    int southBlocked = horzWall[currPos->row + 1][currPos->col];
    int westBlocked = vertWall[currPos->row][currPos->col];

    if (row != 0 && Manhattans[row - 1][col] < Manhattans[row][col] && !northBlocked)
        nextHead = NORTH;
    if (col != 15 && Manhattans[row][col + 1] < Manhattans[row][col] && !eastBlocked)
        nextHead = EAST;
    if (row != 15 && Manhattans[row + 1][col] < Manhattans[row][col] && !southBlocked)
        nextHead = SOUTH;
    if (col != 0 && Manhattans[row][col - 1] < Manhattans[row][col] && !westBlocked)       // Find next heading
        nextHead = WEST;

    if (nextHead == -1)                     // If no path available, then recalculta
    {
        recalculate();
        return IDLE;
    }

    if (nextHead == currHead)               // If next heading is in same direction, move forward
    {
        switch (currHead)
        {
        case NORTH:
            currPos->row--;
            break;
        case EAST:
            currPos->col++;
            break;
        case SOUTH:
            currPos->row++;
            break;
        case WEST:
            currPos->col--;
            break;
        }
        return FORWARD;
    }

    if ((nextHead - currHead) % 2 == 0)         // If next heading is in opposite direction, turn right
    {
        if (currHead == WEST)
            currHead = NORTH;
        else
            currHead++;
        return RIGHT;
    }

    if ((nextHead - currHead) == 1 || nextHead - currHead == -3)  // If next heading is right, turn right
    {
        if (currHead == WEST)
            currHead = NORTH;
        else
            currHead++;
        return RIGHT;
    }

    if (currHead == NORTH)  // else, turn left
        currHead = WEST;
    else
        currHead--;
    return LEFT;

    //for (int i = 0; i < 16; i++)
    //{
    //    for (int j = 0; j < 16; j++)
    //    {
    //        insertQueue(newCell(i, j));
    //        API_setColor(i, j, 'g');
    //    }
    //}
    //for (int i = 0; i < 16; i++)
    //{
    //    for (int j = 0; j < 16; j++)
    //    {
    //        insertQueue(newCell(i, j));
    //        API_setColor(i, j, 'r');
    //    }
    //}

    //while (queueStart != queueEnd)
    //{
    //    API_setColor(queueFront()->row, queueFront()->col, 'b');
    //    popQueueFront();
    //}
    //
    //for (int i = 0; i < 16; i++)
    //{
    //    for (int j = 0; j < 16; j++)
    //    {
    //        insertQueue(newCell(i, j));
    //        API_setColor(i, j, 'y');
    //    }
    //}

    //while (queueStart != queueEnd)
    //{
    //    API_setColor(queueFront()->row, queueFront()->col, 'g');
    //    popQueueFront();
    //}
}
