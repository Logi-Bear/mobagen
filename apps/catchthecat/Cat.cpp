#include "Cat.h"
#include "World.h"

// Steps to whichever neighbor rates best (see SpotRating::isBetterThan in Agent.cpp).
Point2D Cat::Move(CatWorld* world) {
  CatStep best = bestStepForCat(*world);
  Point2D move = best.cell;

  // Completely surrounded: every move loses, so just return any cell on the board.
  if (!best.canMove)
    for (Point2D neighbor : CatWorld::neighbors(world->getCat()))
      if (world->isValidPosition(neighbor)) move = neighbor;

  world->lastMove = move;
  return move;
}
