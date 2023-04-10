#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  buildPointMasses();
  buildSprings();
}

void Cloth::buildPointMasses() {
    double di = height / num_height_points;
    double dj = width / num_width_points;
    for (int i = 0; i < num_height_points; i++) {
        for (int j = 0; j < num_width_points; j++) {
            Vector3D location;
            if (orientation == HORIZONTAL) {
                location = Vector3D(j * dj, 1, di * i);
            } else {
                double offset = ((double) ::rand())/RAND_MAX * 0.002 - 0.001;
                location = Vector3D(dj * j, di * i, offset);
            }
            PointMass p = PointMass(location, false);
            point_masses.emplace_back(p);
        }
    }
    for (vector<int> pin : pinned) {
        point_masses[pin[0] * num_width_points + pin[1]].pinned = true;
    }
}

void Cloth::buildSprings() {
    for (int i = 0; i < num_height_points; i++) {
        for (int j = 0; j < num_width_points; j++) {
            //add structural constraints
            addSpring(j, i, j - 1, i, CGL::STRUCTURAL);
            addSpring(j, i, j, i - 1, CGL::STRUCTURAL);
            //add shearing constraints
            addSpring(j , i, j - 1, i - 1, CGL::SHEARING);
            addSpring(j, i, j - 1, i + 1, CGL::SHEARING);
            //add bending constraints
            addSpring(j, i, j - 2, i, CGL::BENDING);
            addSpring(j, i, j, i - 2, CGL::BENDING);
        }
    }
}

void Cloth::addSpring(int wPos, int hPos, int dw, int dh, e_spring_type sType) {
    if (dh >= 0 && dh < num_width_points && dw >= 0 && dw < num_height_points) {
        Spring s = Spring(&point_masses[wPos * num_width_points + hPos],
                          &point_masses[dw * num_width_points + dh],
                          sType);
        springs.emplace_back(s);
    }
}

void Cloth::removeForces() {
    for (PointMass p: point_masses) {
        p.forces = Vector3D(0);
    }
}

void Cloth::computeTotalForces(ClothParameters *cp, vector<Vector3D> external_accelerations, double mass) {
    removeForces();
    Vector3D ext = Vector3D(0);
    for (Vector3D &accel: external_accelerations) {
        ext += accel * mass;
    }
    for (PointMass &p: point_masses) {
        p.forces = ext;
    }
    for (Spring &s: springs) {
        switch (s.spring_type) {
            case CGL::STRUCTURAL:
                if (!cp->enable_structural_constraints) {
                    continue;
                }
            case CGL::SHEARING:
                if (!cp->enable_shearing_constraints) {
                    continue;
                }
            case CGL::BENDING:
                if (!cp->enable_bending_constraints) {
                    continue;
                }
        }
        Vector3D aTob = s.pm_a->position - s.pm_b->position;
        double SpringForce = 0.2 * cp->ks * ((aTob).norm() - s.rest_length);
        s.pm_a->forces -= aTob.unit() * SpringForce;
        s.pm_b->forces += aTob.unit() * SpringForce;
    }
}

void Cloth::computeVerletIntegration(ClothParameters *cp, double mass, double delta_t) {
    for (PointMass &p: point_masses) {
        if (!p.pinned) {
            Vector3D oldLoc = p.position;
            p.position = p.position + (1. - cp->damping/100.) * (p.position - p.last_position) + (p.forces / mass) * delta_t * delta_t;
            p.last_position = oldLoc;
        }
    }
}

void Cloth::constrainPositionUpdates(ClothParameters *cp) {
    for (Spring &s: springs) {
        switch (s.spring_type) {
            case CGL::STRUCTURAL:
                if (!cp->enable_structural_constraints) {
                    continue;
                }
            case CGL::SHEARING:
                if (!cp->enable_shearing_constraints) {
                    continue;
                }
            case CGL::BENDING:
                if (!cp->enable_bending_constraints) {
                    continue;
                }
        }
        Vector3D directionVec = s.pm_a->position - s.pm_b->position;
        double springLength = directionVec.norm();
        if (springLength > 1.1 * s.rest_length) {
            //case 1: both point masses are pinned
            if (s.pm_a->pinned && s.pm_b->pinned) {
                continue;
            }
                //case 2: only point mass a is pinned
            else if (s.pm_a->pinned) {
                s.pm_b->position += directionVec.unit() * (springLength - 1.1 * s.rest_length);
            }
                //case 3: only point mass b is pinned
            else if (s.pm_b->pinned) {
                s.pm_a->position -= directionVec.unit() * (springLength - 1.1 * s.rest_length);
            } else {
                s.pm_b->position += 0.5 * directionVec.unit() * (springLength - 1.1 * s.rest_length);
                s.pm_a->position -= 0.5 * directionVec.unit() * (springLength - 1.1 * s.rest_length);
            }
        }
    }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  computeTotalForces(cp, external_accelerations, mass);

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  computeVerletIntegration(cp, mass, delta_t);

  // TODO (Part 4): Handle self-collisions.


  // TODO (Part 3): Handle collisions with other primitives.


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  constrainPositionUpdates(cp);

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.

  return 0.f; 
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
