#include "../World.h"
#include "../SeededRandom.h"
#include "RecursiveBacktrackerExample.h"
#include <climits>

static const Color32 kCurrent = {0.5f, 1.0f, 0.5f, 1.0f};
static const Color32 kStackHighlight = {1.0f, 0.5f, 0.5f, 1.0f};
static const Color32 kWhite = {1.0f, 1.0f, 1.0f, 1.0f};

// Recursive backtracker, in FORMAL units: (0, 0) is the top-left cell, x grows
// right, y grows down. The caller seeds SeededRandom before the first Step;
// every decision consumes the seed in order, so the maze is deterministic.
//
// Procedure per Step, on the cell at the top of the path stack:
//   1. mark it visited;
//   2. list its visitable (unvisited) neighbors in clockwise order starting
//      from the top: UP, RIGHT, DOWN, LEFT (getVisitables does this);
//   3. none        -> dead end: pop the stack (backtrack). Empty stack = done;
//   4. exactly one -> move to it, do not consume a random number;
//   5. two or more -> consume SeededRandom::next() and pick
//      next() % visitableCount;
//   6. moving opens the wall between the two cells
//      (World::SetNorth/SetEast/SetSouth/SetWest with false).

void RecursiveBacktrackerExample::Clear(World* world) {
  // todo: reset the walk
  // hint:
  //   clear visited and the path stack, then start the walk at the
  //   top-left cell: stack.push_back({0, 0})
  // begin solution
  visited.clear();
  stack.clear();

  stack.emplace_back(0, 0);
  // end solution
}

bool RecursiveBacktrackerExample::Step(World* w) {
  // todo: implement one iteration of the recursive backtracker
  // hint:
  //   empty stack  -> the maze is done, return false
  //   otherwise, on the cell at the top of the stack:
  //   1. mark it visited;
  //   2. list its visitable neighbors with getVisitables
  //      (already in clockwise order: UP, RIGHT, DOWN, LEFT);
  //   3. none        -> dead end: pop the stack (backtrack);
  //   4. exactly one -> move to it, do not consume a random number;
  //   5. two or more -> consume SeededRandom::next() and pick
  //      next() % visitables.size();
  //   moving = opening the wall between the two cells:
  //     UP    -> w->SetNorth(current, false)
  //     RIGHT -> w->SetEast(current, false)
  //     DOWN  -> w->SetSouth(current, false)
  //     LEFT  -> w->SetWest(current, false)
  //   return true while there is still work (stack not empty after the move)
  // begin solution

  if (!stack.empty())
  {
    Point2D const point = stack.back();
    visited[point.x][point.y] = true;

    w->SetNodeColor(point, kCurrent);

    for (size_t i = 0; i + 1 < stack.size(); ++i)
    {
      w->SetNodeColor(stack[i], kStackHighlight);
    }

    std::vector<Point2D> const visitables = getVisitables(w, point);

    if (visitables.empty())
    {
      w->SetNodeColor(point, kWhite);
      stack.pop_back();
      if (!stack.empty())
      {
        w->SetNodeColor(stack.back(), kCurrent);
      }
    }
    else
    {
      Point2D next;
      if (visitables.size() == 1)
      {
        next = visitables.back();
      }
      else if (visitables.size() >= 2)
      {
        next = visitables.at(SeededRandom::next() % visitables.size());
      }
      stack.emplace_back(next);

      if (next.y < point.y)       {w->SetNorth(point, false);}
      else if (next.x > point.x)  {w->SetEast(point, false);}
      else if (next.y > point.y)  {w->SetSouth(point, false);}
      else if (next.x < point.x)  {w->SetWest(point, false);}
    }
    if (!stack.empty())
    {
      return true;
    }
  }
  // end solution
  return false;
}

std::vector<Point2D> RecursiveBacktrackerExample::getVisitables(World* w, const Point2D& point) {
  // todo: list the unvisited neighbors of point, in clockwise order
  // hint:
  //   candidates in order: UP {x, y-1}, RIGHT {x+1, y}, DOWN {x, y+1}, LEFT {x-1, y}
  //   keep a candidate only if it is inside the grid
  //   (0 <= x < w->GetWidth(), 0 <= y < w->GetHeight()) and not visited
  // begin solution

  std::vector<Point2D> visitables;

  Point2D const up{point.x, point.y - 1};
  Point2D const right{point.x + 1, point.y};
  Point2D const down{point.x, point.y + 1};
  Point2D const left{point.x - 1, point.y};

    if (!visited[up.x][up.y])
    {
      if (0 <= up.x && up.x < w->GetWidth() && 0 <= up.y && up.y < w->GetHeight()) {
        visitables.push_back(up);
      }
    }
    if (!visited[right.x][right.y])
    {
      if (0 <= right.x && right.x < w->GetWidth() && 0 <= right.y && right.y < w->GetHeight()) {
        visitables.push_back(right);
      }
    }
    if (!visited[down.x][down.y])
    {
      if (0 <= down.x && down.x < w->GetWidth() && 0 <= down.y && down.y < w->GetHeight()) {
        visitables.push_back(down);
      }
    }
    if (!visited[left.x][left.y])
    {
      if (0 <= left.x && left.x < w->GetWidth() && 0 <= left.y && left.y < w->GetHeight()) {
        visitables.push_back(left);
      }
    }
  // end solution
  return visitables;
}
