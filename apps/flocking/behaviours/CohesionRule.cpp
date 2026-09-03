#include "CohesionRule.h"
#include <glm/glm.hpp>

glm::vec2 CohesionRule::computeForce(const std::vector<BoidView>& neighborhood, const BoidView& boid) {
  glm::vec2 cohesionForce(0.f);

  // glm::length(vec) returns the length of a vector,
  // glm::normalize(vec) returns the normalized vector (length 1) in the same direction as vec.

  // begin solution
  glm::vec2 centerMass (0.0f, 0.0f);
  float boidCount = 0;
  if (!neighborhood.empty()) {
    for (BoidView neighbor : neighborhood)
    {
      if (&neighbor == &boid)
      {
        continue;
      }
      centerMass += neighbor.position;
      boidCount++;
    }
    if (boidCount > 0)
    {
      centerMass /= boidCount;
      cohesionForce = glm::normalize(centerMass - boid.position);
    }
  }
  // end solution

  return cohesionForce;
}
