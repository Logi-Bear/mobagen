#include "Cat.h"
#include "World.h"

#include <cstdlib>

// How long the search may think per move. Testing showed no difference between 3ms and 40ms,
// and every millisecond costs points through the tournament's time penalty.
static const int THINKING_TIME_MS = 3;
static const int FENCE_THINKING_TIME_MS = 10;  // for planning around a predictable fence builder

// How many more walls than "random chance" the edge needs before we decide the
// catcher is building a fence along it.
static const double EXTRA_EDGE_WALLS_FOR_FENCE = 2;

static bool catcherIsFencingTheEdge(const CatWorld& world) {
  int half = world.getWorldSideSize() / 2;
  Point2D cat = world.getCat();
  int edgeCells = 0, edgeWalls = 0, middleCells = 0, middleWalls = 0;

  for (int row = -half; row <= half; row++)
    for (int column = -half; column <= half; column++) {
      Point2D cell{column, row};
      bool isWall = world.getContent(cell);
      bool farFromCat = std::abs(column - cat.x) > 3 || std::abs(row - cat.y) > 3;
      if (world.catWinsOnSpace(cell)) {
        edgeCells++;
        if (isWall) edgeWalls++;
      } else if (farFromCat) {
        middleCells++;
        if (isWall) middleWalls++;
      }
    }

  double randomWallDensity = (double)middleWalls / middleCells;
  double expectedEdgeWalls = edgeCells * randomWallDensity;
  return edgeWalls - expectedEdgeWalls >= EXTRA_EDGE_WALLS_FOR_FENCE;
}

// Verify move because an invalid move loses instantly.
static Point2D legalMoveOrFallback(const CatWorld& world, Point2D wanted) {
  Point2D cat = world.getCat();
  bool wantedIsLegal = world.isValidPosition(wanted) && !world.getContent(wanted) && CatWorld::isNeighbor(cat, wanted);
  if (wantedIsLegal) return wanted;

  for (Point2D neighbor : CatWorld::neighbors(cat))
    if (world.isValidPosition(neighbor) && !world.getContent(neighbor)) return neighbor;
  for (Point2D neighbor : CatWorld::neighbors(cat))
    if (world.isValidPosition(neighbor)) return neighbor;  // surrounded: every move loses anyway
  return cat;
}

Point2D Cat::Move(CatWorld* world) {
  SearchBoard board(*world);
  int chosenCell;

  if (catcherIsFencingTheEdge(*world)) {
    // Race to the edge against a catcher that can only wall edge cells.
    chosenCell = bestStepAgainstTemplateCatcher(board, FENCE_THINKING_TIME_MS);
    if (chosenCell == OFF_BOARD) chosenCell = bestStepForCat(board).cell;
  } else {
    // This catcher fights up close: think a few moves ahead.
    chosenCell = GameSearch(board, THINKING_TIME_MS).bestStep();
  }

  Point2D wanted = chosenCell == OFF_BOARD ? world->getCat() : board.toPoint(chosenCell);
  Point2D move = legalMoveOrFallback(*world, wanted);
  world->lastMove = move;
  return move;
}
