#include "SeparationRule.h"
#include "imgui.h"
#include <glm/glm.hpp>

glm::vec2 SeparationRule::computeForce(const std::vector<BoidView>& boids, int selfIndex) {
  glm::vec2 separatingForce(0.f);

  // the header have the radius member variable, which is how far this boid perceives others: boids further away than radius are not neighbors.
  // glm::length(vec) returns the length of a vector,
  // glm::normalize(vec) returns the normalized vector (length 1) in the same direction as vec.
  // dividing the away direction by the distance makes the force inversely proportional to it: stronger when the boids are closer together
  // Bonus: use spatial hashing to avoid O(n^2) complexity. Implement that on World.

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
      if (distance > 0.0001f && distance < desiredMinimalDistance)
      {
        separatingForce += direction * (desiredMinimalDistance / distance);
      }
    }
  }

  // end solution

  return separatingForce;
}
bool SeparationRule::drawImguiRuleExtra() {
  bool valueHasChanged = false;
  if (ImGui::DragFloat("Radius", &radius, 0.05f)) {
    valueHasChanged = true;
  }
  return valueHasChanged;
}

void SeparationRule::drawRadius(const BoidView& boid, ImDrawList* dl) const {
  dl->AddCircle({boid.position.x, boid.position.y}, radius,
                IM_COL32(static_cast<int>(debugColor.r * 255), static_cast<int>(debugColor.g * 255), static_cast<int>(debugColor.b * 255), 64), 32);
}