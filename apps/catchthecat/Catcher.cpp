#include "Catcher.h"
#include "World.h"

// How long the catcher may think per move. The catcher has far more possible moves
// than the cat (dozens of walls vs. 6 steps), so deeper search pays off: in testing,
// 60 ms allowed less than half the escapes of 40 ms, and still leaves 40 ms of
// headroom under the tournament's 100 ms limit.
static const int THINKING_TIME_MS = 50;

// The catcher uses the same search as the cat, from the other side: it plays the
// next few moves out in its head and places the wall that leaves the cat worst off
Point2D Catcher::Move(CatWorld* world) {
  SearchBoard board(*world);
  int wall = GameSearch(board, THINKING_TIME_MS).bestWall();

  // Verify move because an invalid move loses instantly.
  bool wallIsLegal = wall != OFF_BOARD && board.isOpen(wall) && wall != board.catCell();
  if (!wallIsLegal)
    for (int cell = 0; cell < board.cellCount(); cell++)
      if (cell != board.catCell() && board.isOpen(cell)) wall = cell;

  Point2D move = board.toPoint(wall);
  world->lastMove = move;
  return move;
}
