#ifndef COLLISION_MATH_H
#define COLLISION_MATH_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include "../include/definitions.h"
#include "../include/Geometry.h"

/**** Object that stores x, dx, and d^2x to be used in approximating the solution of x(t) ****/
typedef struct {
  Vector3 position;
  Vector3 velocity;
  Vector3 acceleration;
  double mass;
  double radius;
  Int3 wall;
} Object;               // 80 Bytes

/**** Hashtable that stores the index of particles in each bucket for collision map ****/
typedef struct Map{
  int count;
  int obj_index;
  struct Map *next;
} Map;                  // 16 Bytes

typedef Vector3 (*PhysicsFunc)(Vector3, Vector3);

typedef struct {
  Object *objects;
  double dt; 
  int start;
  int count; 
  PhysicsFunc physics;
} ForceArgs;            // 40 Bytes

typedef struct {
  Map **map;
  Cube cube;
  Object *objects;
  int axis_count;
  pthread_barrier_t *barrier;
} CollisionArgs;       // 

Object *
InitializeObjects(const int count, const double radius);

Vector3
Gravity(const Vector3 position, const Vector3 velocity);

Vector3
Hyperbolic(const Vector3 position, const Vector3 velocity);

void
ApplyForce(Object objects[], const double dt, const double count, PhysicsFunc physics);

int
IndexCalc(const Int3 index, const int axis_ct);

Int3
DecomposeIndex(const int index, const int axis_ct);

int
InsertObj(Map *map[], const int grid_index, const int obj_index);

Map **
CreateMap(Object objects[], const Cube cube, 
          const int axis_ct, const int particle_ct, int *status);

int *
CalculateIterator(Int3 (*scan)[3][3], Int3 center, const int axis_ct);

void
ProcessWall(Cube cube, Object objects[], const Int3 bad_index, const int obj_index, const int axis_ct);

void
HandleCollision(Object *src, Object *deflecting);

void
Overlap(double min, double max, double *position, double radius);

void
CollisionCall(Cube cube, Object objects[], 
              const int partition_ct, const int particle_ct, const int axis_ct);

void
DestroyMap(Map *map[], const int size);

void
PrintPositions(const Object objects[], const int count);

#endif // COLLISION_MATH_H