#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
  if (radius - (pm.position - origin).norm() > 0) {
      Vector3D directionVec = (pm.position - origin).unit();
      Vector3D intersectionPt = origin + directionVec * radius;
      Vector3D correctionVec = intersectionPt - pm.last_position;
      pm.position = pm.last_position + (1. - friction) * correctionVec;
  }

}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
