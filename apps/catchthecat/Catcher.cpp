#include "Catcher.h"
#include "World.h"

// Only walls this close to the cat are worth trying.
static const int WALL_SEARCH_RADIUS = 6;

// Tries a wall on every open cell near the cat and keeps the wall that leaves that best step the worst.
Point2D Catcher::Move(CatWorld* world) {
  SearchBoard board(*world);
  int cat = board.catCell();
  bool catIsSealedIn = board.shortestEscapeSteps()[cat] == NOT_REACHABLE;
  int searchRadius = catIsSealedIn ? 1 : WALL_SEARCH_RADIUS;  // sealed in: just take away its roomiest step
  std::vector<int> stepsFromCat = board.stepsFrom({cat}, 1);

  int bestWall = OFF_BOARD;
  SpotRating catsBestReply;
  for (int cell = 0; cell < board.cellCount(); cell++) {
    if (cell == cat || !board.isOpen(cell) || stepsFromCat[cell] > searchRadius) continue;

    board.placeWall(cell);
    CatStep reply = bestStepForCat(board);
    board.removeWall(cell);

    if (!reply.canMove) {  // this wall traps the cat right now
      bestWall = cell;
      break;
    }
    if (bestWall == OFF_BOARD || catsBestReply.isBetterThan(reply.rating)) {
      bestWall = cell;
      catsBestReply = reply.rating;
    }
  }

  // Nothing near the cat to block
  if (bestWall == OFF_BOARD)
    for (int cell = 0; cell < board.cellCount(); cell++)
      if (cell != cat && board.isOpen(cell)) bestWall = cell;

  Point2D move = board.toPoint(bestWall);
  world->lastMove = move;
  return move;
}
