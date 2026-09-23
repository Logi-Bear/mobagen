#include "HuntAndKillExample.h"
#include "../World.h"
#include "../SeededRandom.h"
#include <climits>

static const Color32 kCurrent = {0.5f, 1.0f, 0.5f, 1.0f};
static const Color32 kStackHighlight = {0.75f, 0.75f, 0.75f, 1.0f};
static const Color32 kWhite = {1.0f, 1.0f, 1.0f, 1.0f};

static void openWallBetween(World* w, const Point2D& from, const Point2D& to) {
  if (to.y < from.y)      { w->SetNorth(from, false); }
  else if (to.x > from.x) { w->SetEast(from, false); }
  else if (to.y > from.y) { w->SetSouth(from, false); }
  else if (to.x < from.x) { w->SetWest(from, false); }
}

bool HuntAndKillExample::Step(World* w) {
  if (!stack.empty()) {
    // --- walk phase ---
    Point2D const current = stack.back();
    visited[current.y][current.x] = true;
    w->SetNodeColor(current, kCurrent);

    std::vector<Point2D> const visitables = getVisitables(w, current);

    if (visitables.empty()) {
      // dead end: this walk is over. Dim the final cell and leave the
      // stack empty to signal "hunt phase" on the next Step call.
      w->SetNodeColor(current, kWhite);
      stack.pop_back();
    } else {
      Point2D next;
      if (visitables.size() == 1) {
        next = visitables.front();
      } else {
        next = visitables.at(SeededRandom::next() % visitables.size());
      }

      openWallBetween(w, current, next);
      visited[next.y][next.x] = true;

      // the old current cell is no longer active; dim it, then move on
      w->SetNodeColor(current, kWhite);
      w->SetNodeColor(next, kCurrent);
      stack.back() = next;
    }

    return true;
  }

  //hunt phase
  bool anyVisited = false;
  for (auto const& row : visited) {
    for (auto const& cell : row.second) {
      if (cell.second) { anyVisited = true; break; }
    }
    if (anyVisited) break;
  }

  if (!anyVisited) {
    // very first call: nothing to connect to yet, just seed the walk
    Point2D const start = randomStartPoint(w);
    if (start.x == INT_MAX) { return false; } // shouldn't happen on an empty grid, but just in case
    visited[start.y][start.x] = true;
    w->SetNodeColor(start, kCurrent);
    stack.push_back(start);
    return true;
  }

  for (int y = 0; y < w->GetHeight(); ++y) {
    for (int x = 0; x < w->GetWidth(); ++x) {
      // highlight the cell the hunt is currently checking
      w->SetNodeColor(Point2D{x, y}, kStackHighlight);

      if (visited[y][x]) continue;

      Point2D const candidate{x, y};
      std::vector<Point2D> const visitedNeighbors = getVisitedNeighbors(w, candidate);
      if (visitedNeighbors.empty()) continue;

      Point2D neighbor;
      if (visitedNeighbors.size() == 1) {
        neighbor = visitedNeighbors.front();
      } else {
        neighbor = visitedNeighbors.at(SeededRandom::next() % visitedNeighbors.size());
      }

      openWallBetween(w, neighbor, candidate);
      visited[candidate.y][candidate.x] = true;
      w->SetNodeColor(candidate, kCurrent);
      stack.push_back(candidate);
      return true;
    }
  }

  // no unvisited cell has a visited neighbor left: maze complete
  return false;
}

void HuntAndKillExample::Clear(World* world) {
  visited.clear();
  stack.clear();

  for (int i = 0; i < world->GetHeight(); i++) {
    for (int j = 0; j < world->GetWidth(); j++) {
      visited[i][j] = false;
    }
  }
}
Point2D HuntAndKillExample::randomStartPoint(World* world) {
  // Todo: improve this if you want
  for (int y = 0; y < world->GetHeight(); y++)
    for (int x = 0; x < world->GetWidth(); x++)
      if (!visited[y][x]) return {x, y};
  return {INT_MAX, INT_MAX};
}

std::vector<Point2D> HuntAndKillExample::getVisitables(World* w, const Point2D& p) {
  std::vector<Point2D> visitables;

  Point2D const up{p.x, p.y - 1};
  Point2D const right{p.x + 1, p.y};
  Point2D const down{p.x, p.y + 1};
  Point2D const left{p.x - 1, p.y};

  if (0 <= up.x && up.x < w->GetWidth() && 0 <= up.y && up.y < w->GetHeight() && !visited[up.y][up.x]) {
    visitables.push_back(up);
  }
  if (0 <= right.x && right.x < w->GetWidth() && 0 <= right.y && right.y < w->GetHeight() && !visited[right.y][right.x]) {
    visitables.push_back(right);
  }
  if (0 <= down.x && down.x < w->GetWidth() && 0 <= down.y && down.y < w->GetHeight() && !visited[down.y][down.x]) {
    visitables.push_back(down);
  }
  if (0 <= left.x && left.x < w->GetWidth() && 0 <= left.y && left.y < w->GetHeight() && !visited[left.y][left.x]) {
    visitables.push_back(left);
  }

  return visitables;
}

std::vector<Point2D> HuntAndKillExample::getVisitedNeighbors(World* w, const Point2D& p) {
  std::vector<Point2D> deltas = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};
  std::vector<Point2D> neighbors;

  for (auto const& d : deltas) {
    Point2D const n{p.x + d.x, p.y + d.y};
    if (0 <= n.x && n.x < w->GetWidth() && 0 <= n.y && n.y < w->GetHeight() && visited[n.y][n.x]) {
      neighbors.push_back(n);
    }
  }

  return neighbors;
}
