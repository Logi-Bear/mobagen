#include "SeparationRule.h"
#include "imgui.h"
#include <glm/glm.hpp>

glm::vec2 SeparationRule::computeForce(const std::vector<BoidView>& neighborhood, const BoidView& boid) {
  glm::vec2 separatingForce(0.f);

  // the header have the desiredMinimalDistance member variable, which is the distance that the boids should try to maintain from each other.
  // glm::length(vec) returns the length of a vector,
  // glm::normalize(vec) returns the normalized vector (length 1) in the same direction as vec.
  // multiply by (desiredMinimalDistance / distance) is the proportionality factor that makes the force stronger when the boids are closer together, and weaker when they are farther apart.

  // begin solution
  if (!neighborhood.empty())
  {
    for (BoidView neighbor : neighborhood)
    {
      if (&neighbor == &boid)
      {
        continue;
      }
      glm::vec2 difference = boid.position - neighbor.position;
      glm::vec2 direction = glm::normalize(difference);
      float distance = glm::length(difference);
      if (distance > 0)
      {
        separatingForce += direction * (desiredMinimalDistance / distance) * (weight / distance);
      }
    }
  }


  // end solution

  return separatingForce;
}

bool SeparationRule::drawImguiRuleExtra() {
  bool valueHasChanged = false;
  if (ImGui::DragFloat("Desired Separation", &desiredMinimalDistance, 0.05f)) {
    valueHasChanged = true;
  }
  return valueHasChanged;
}
