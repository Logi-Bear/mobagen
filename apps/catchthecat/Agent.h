#ifndef AGENT_H
#define AGENT_H

#include <glm/glm.hpp>
#include <functional>
#include <vector>

// Point2D is now glm::ivec2 — same x,y interface, no OOP wrapper needed.
using Point2D = glm::ivec2;

// Hash specialization so Point2D (= glm::ivec2) works in unordered containers.
namespace std {
  template <> struct hash<glm::ivec2> {
    std::size_t operator()(const glm::ivec2& v) const noexcept {
      std::size_t seed = std::hash<int>{}(v.x);
      seed ^= std::hash<int>{}(v.y) + 0x9e3779b9u + (seed << 6) + (seed >> 2);
      return seed;
    }
  };
}  // namespace std

class CatWorld;

class Agent {
public:
  explicit Agent() = default;
  virtual ~Agent() = default;

  virtual Point2D Move(CatWorld*) = 0;

  std::vector<Point2D> generatePath(CatWorld* w);

protected:
static const int NOT_REACHABLE = 1000000;

  // One number for every cell on the board, stored in a flat list laid out like worldState().
  // Lets numbers on the board to be given values for multiple uses
  class CellNumbers {
  public:
    CellNumbers(const CatWorld& world, int startingValue);
    int get(Point2D cell) const;
    void set(Point2D cell, int value);
    int countCellsNotEqualTo(int value) const;

  private:
    int indexOf(Point2D cell) const;
    int side;
    std::vector<int> numbers;
  };

  // Everything the cat cares about when deciding where to step.
  struct SpotRating {
    bool isEdge = false;        // stepping here wins the game
    bool canReachEdge = false;  // false means the cat is sealed in
    int room = 0;               // open cells the cat could still roam (only matters when sealed in)
    int guaranteedSteps = NOT_REACHABLE;
    int shortestSteps = NOT_REACHABLE;
    int openNeighborCount = 0;

    // Compares the rules in order of importance; the first difference decides.
    bool isBetterThan(const SpotRating& other) const;
  };

  struct CatStep {
    bool canMove = false;  // false means the cat is completely surrounded
    Point2D cell = {0, 0};
    SpotRating rating;
  };

  static bool isOpen(const CatWorld& world, Point2D cell);
  static std::vector<Point2D> allCells(const CatWorld& world);
  static std::vector<Point2D> openNeighbors(const CatWorld& world, Point2D cell);
  static std::vector<Point2D> openEdgeCells(const CatWorld& world);

  // Spreads outward from the start cells and counts how many steps away every open cell is.
  // A cell only counts as reached once `neighborsNeeded` of its neighbors have been reached.
  static CellNumbers countStepsFrom(const CatWorld& world, const std::vector<Point2D>& startCells, int neighborsNeeded);

  // Fewest steps from each cell to the edge, if nobody gets in the way.
  static CellNumbers shortestEscapeSteps(const CatWorld& world);

  // Fewest steps from each cell to the edge even if the catcher blocks the cat's
  // best move every turn. Since one way forward always gets blocked, a cell only
  // counts if TWO of its neighbors lead out.
  static CellNumbers guaranteedEscapeSteps(const CatWorld& world);

  // How many open cells the cat could still reach from this cell.
  static int roomAround(const CatWorld& world, Point2D cell);

  // The best step the cat can take on this board, and how good it is.
  static CatStep bestStepForCat(const CatWorld& world);
};

#endif  // AGENT_H
