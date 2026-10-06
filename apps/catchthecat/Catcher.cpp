#include "Catcher.h"
#include "World.h"

// Only walls this close to the cat are worth trying.
static const int WALL_SEARCH_RADIUS = 6;

// A copy of the board with one extra wall, for trying a move out.
static CatWorld boardWithWallAt(const CatWorld& world, Point2D cell) {
  int side = world.getWorldSideSize();
  std::vector<bool> walls = world.worldState();
  walls[(cell.y + side / 2) * side + (cell.x + side / 2)] = true;
  return CatWorld(side, true, world.getCat(), walls);
}

// Tries a wall on every open cell near the cat and keeps the wall that leaves that best step the worst.
Point2D Catcher::Move(CatWorld* world) {
  Point2D cat = world->getCat();
  bool catIsSealedIn = shortestEscapeSteps(*world).get(cat) == NOT_REACHABLE;
  int searchRadius = catIsSealedIn ? 1 : WALL_SEARCH_RADIUS;  // sealed in: just take away its roomiest step
  CellNumbers stepsFromCat = countStepsFrom(*world, {cat}, 1);

  bool foundWall = false;
  Point2D bestWall = cat;
  SpotRating catsBestReply;
  for (Point2D cell : allCells(*world)) {
    if (cell == cat || !isOpen(*world, cell) || stepsFromCat.get(cell) > searchRadius) continue;

    CatStep reply = bestStepForCat(boardWithWallAt(*world, cell));

    if (!reply.canMove) {  // this wall traps the cat right now
      bestWall = cell;
      foundWall = true;
      break;
    }
    if (!foundWall || catsBestReply.isBetterThan(reply.rating)) {
      bestWall = cell;
      catsBestReply = reply.rating;
      foundWall = true;
    }
  }

  // Nothing near the cat to block
  if (!foundWall)
    for (Point2D cell : allCells(*world))
      if (cell != cat && isOpen(*world, cell)) bestWall = cell;

  world->lastMove = bestWall;
  return bestWall;
}
