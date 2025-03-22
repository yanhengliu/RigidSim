// ----------------------------------------------------------------------------
// RigidSolver.hpp
//
//  Created on: 18 Dec 2020
//      Author: Kiwon Um
//        Mail: kiwon.um@telecom-paris.fr
//
// Description: Simple Rigid Body Solver (DO NOT DISTRIBUTE!)
//
// Copyright 2020-2024 Kiwon Um
//
// The copyright to the computer program(s) herein is the property of Kiwon Um,
// Telecom Paris, France. The program(s) may be used and/or copied only with
// the written permission of Kiwon Um or in accordance with the terms and
// conditions stipulated in the agreement/contract under which the program(s)
// have been supplied.
// ----------------------------------------------------------------------------

#ifndef _RIGIDSOLVER_HPP_
#define _RIGIDSOLVER_HPP_

#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <iostream>
#include "Vector3.hpp"
#include "Matrix3x3.hpp"

// A helper function to compute the cross product of two 3D vectors.
// We define it here as a free function for clarity.
inline Vec3f crossProduct(const Vec3f &a, const Vec3f &b)
{
  // Cross product: a x b
  return Vec3f(
    a[1]*b[2] - a[2]*b[1],
    a[2]*b[0] - a[0]*b[2],
    a[0]*b[1] - a[1]*b[0]
  );
}

struct BodyAttributes {
  BodyAttributes()
    : X(0, 0, 0), R(Mat3f::I()), P(0, 0, 0), L(0, 0, 0),
      V(0, 0, 0), omega(0, 0, 0), F(0, 0, 0), tau(0, 0, 0),
      q(1.f, 0.f, 0.f, 0.f) // Initialize quaternion as identity
  {}

  // This function returns the model matrix for rendering.
  glm::mat4 worldMat() const {
    return glm::mat4(
      R(0,0), R(1,0), R(2,0), 0,
      R(0,1), R(1,1), R(2,1), 0,
      R(0,2), R(1,2), R(2,2), 0,
      X[0],   X[1],   X[2],   1
    );
  }

  tReal M;       // Mass
  Mat3f I0;      // Inertia tensor in body space
  Mat3f I0inv;   // Inverse of I0
  Mat3f Iinv;    // Inverse inertia tensor in world space

  Vec3f X;       // Position
  Mat3f R;       // Rotation matrix (for rendering)
  Vec3f P;       // Linear momentum
  Vec3f L;       // Angular momentum

  Vec3f V;       // Linear velocity
  Vec3f omega;   // Angular velocity

  Vec3f F;       // Force
  Vec3f tau;     // Torque

  glm::quat q;   // Quaternion to represent orientation

  // Vertices in body space
  std::vector<Vec3f> vdata0;
};

class Box : public BodyAttributes {
public:
  explicit Box(
    tReal w = 1.0,
    tReal h = 1.0,
    tReal d = 1.0,
    tReal dens = 10.0,
    const Vec3f v0 = Vec3f(0, 0, 0),
    const Vec3f omega0 = Vec3f(0, 0, 0))
    : width(w), height(h), depth(d)
  {
    // Initial linear and angular velocity
    V = v0;
    omega = omega0;

    // Compute mass
    M = dens * w * h * d;

    // Compute inertia tensor for a box with center at (0,0,0).
    // Ixx = (1/12)*M*(h^2 + d^2), etc.
    const tReal oneTwelfth = static_cast<tReal>(1.0 / 12.0);
    const tReal Ixx = oneTwelfth * M * (h*h + d*d);
    const tReal Iyy = oneTwelfth * M * (w*w + d*d);
    const tReal Izz = oneTwelfth * M * (w*w + h*h);

    // Fill I0 manually
    I0(0,0) = Ixx;  I0(0,1) = 0.0f; I0(0,2) = 0.0f;
    I0(1,0) = 0.0f; I0(1,1) = Iyy;  I0(1,2) = 0.0f;
    I0(2,0) = 0.0f; I0(2,1) = 0.0f; I0(2,2) = Izz;

    // Precompute the inverse of I0
    I0inv = I0.inverse();
    // Set the current world-space inverse inertia to I0inv initially
    Iinv = I0inv;

    // Define 8 vertices in body space
    vdata0.push_back(Vec3f(-0.5f*w, -0.5f*h, -0.5f*d));
    vdata0.push_back(Vec3f( 0.5f*w, -0.5f*h, -0.5f*d));
    vdata0.push_back(Vec3f( 0.5f*w,  0.5f*h, -0.5f*d));
    vdata0.push_back(Vec3f(-0.5f*w,  0.5f*h, -0.5f*d));
    vdata0.push_back(Vec3f(-0.5f*w, -0.5f*h,  0.5f*d));
    vdata0.push_back(Vec3f( 0.5f*w, -0.5f*h,  0.5f*d));
    vdata0.push_back(Vec3f( 0.5f*w,  0.5f*h,  0.5f*d));
    vdata0.push_back(Vec3f(-0.5f*w,  0.5f*h,  0.5f*d));
  }

  tReal width, height, depth;
};

class RigidSolver {
public:
  explicit RigidSolver(
    BodyAttributes *body0 = nullptr,
    const Vec3f g = Vec3f(0, 0, 0))
    : body(body0), _g(g), _step(0), _sim_t(0)
  {}

  void init(BodyAttributes *body0) {
    body = body0;
    _step = 0;
    _sim_t = 0;
  }

  void step(const tReal dt) {
    std::cout << "t=" << _sim_t << " (dt=" << dt << ")" << std::endl;

    // 1) Compute force and torque
    computeForceAndTorque();

    // 2) Integrate linear momentum and position
    body->P += body->F * dt;
    body->V = body->P / body->M;
    body->X += body->V * dt;

    // 3) Integrate angular momentum and orientation
    body->L += body->tau * dt;

    // Update inverse inertia in world space
    body->Iinv = body->R * body->I0inv * body->R.transpose();
    body->omega = body->Iinv * body->L;

    // Update quaternion by angular velocity
    glm::quat wq(0.0f, body->omega[0], body->omega[1], body->omega[2]);
    glm::quat dq = 0.5f * wq * body->q;
    body->q += dq * static_cast<float>(dt);
    body->q = glm::normalize(body->q);

    // Convert quaternion back to a rotation matrix
    glm::mat3 rot = glm::mat3_cast(body->q);
    for(int i = 0; i < 3; i++){
      for(int j = 0; j < 3; j++){
        body->R(i, j) = rot[i][j];
      }
    }

    // Clear force and torque
    body->F = Vec3f(0, 0, 0);
    body->tau = Vec3f(0, 0, 0);

    ++_step;
    _sim_t += dt;
  }

  BodyAttributes *body;

private:
  void computeForceAndTorque() {
    // Reset force and torque, then add gravity
    body->F = body->M * _g;
    body->tau = Vec3f(0, 0, 0);

    // Apply a one-time instant force at step 1
    if(_step == 1) {
      Vec3f instF(0.15f, 0.25f, 0.03f);
      body->F += instF;

      // Compute torque: tau = r x F
      // r is the world-space position of vertex 0 relative to the center.
      Vec3f r = body->R * body->vdata0[0];
      Vec3f t = crossProduct(r, instF);
      body->tau += t;
    }
  }

  Vec3f _g;      // Gravity
  tIndex _step;  // Simulation step count
  tReal _sim_t;  // Simulation time
};

#endif  /* _RIGIDSOLVER_HPP_ */
