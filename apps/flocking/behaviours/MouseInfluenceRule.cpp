#include "MouseInfluenceRule.h"
#include "imgui.h"

glm::vec2 MouseInfluenceRule::computeForce(const std::vector<BoidView>& neighborhood, const BoidView& boid) {
  glm::vec2 force(0.f);

  // ImGui::IsMouseDown(ImGuiMouseButton_Left) returns true if the left mouse button is currently pressed.
  // ImGui::GetIO().MousePos returns the current mouse position as an ImVec2.
  // glm::length(vec) returns the length of a vector

  // begin solution
  if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
  {
    float influenceDistance = 250;
    ImVec2 imMousePos = ImGui::GetIO().MousePos;
    glm::vec2 mousePos(imMousePos.x, imMousePos.y);
    glm::vec2 difference = boid.position - mousePos;
    glm::vec2 direction = glm::normalize(difference);
    float distance = glm::length(difference);
    float strength = 250;
    if (distance > 0.0001f && distance < influenceDistance)
    {
      if (isRepulsive)
        force = direction * strength;
      else
        force = -direction * strength;
    }
  }


  // end solution

  return force;
}

bool MouseInfluenceRule::drawImguiRuleExtra() {
  bool valueHasChanged = false;

  if (ImGui::RadioButton("Attractive", !isRepulsive)) {
    isRepulsive = false;
    valueHasChanged = true;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Repulsive", isRepulsive)) {
    isRepulsive = true;
    valueHasChanged = true;
  }

  return valueHasChanged;
}
