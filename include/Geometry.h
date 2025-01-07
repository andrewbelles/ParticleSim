#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include "definitions.h"

// Vector Defns
typedef struct {
  int x, y, z;
} Int3;

typedef struct {
 double x, y;
} Vector2;

typedef struct {
  double x, y, z;
} Vector3;

typedef struct {
  double x, y, z, w;
} Vector4;

typedef struct {
  Vector3 min, max;
  double size; 
} Cube;

// Custom malloc with error handling to reduce debug if statements
void*
safe_malloc(size_t size);

// boolean is or isn't prime
int
is_prime(const int num);

// quake 3 q_rqrt
float
Q_rsqrt(float number);

// Normalizes vectors using q3 
Vector3
Q_normalize(Vector3 a);

Vector3
normalize(Vector3 a);

// Returns a random value between 2.5 and 5
double
randomPosition();

// Returns a vector3 of 3 randomPosition()'s
Vector3
randomVector();

// Vector operations
Vector3
addVectors(Vector3 a, const Vector3 b);

Vector3
addScalar(const Vector3 a, const double scalar);

Vector3
subtractVectors(const Vector3 a, const Vector3 b);

Vector3
subtractScalar(const Vector3 a, const double scalar);

Vector3 
scaleVector(const Vector3 a, const double scalar);

Vector3
crossProduct(Vector3 a, Vector3 b);

double
dotProduct(const Vector3 aT, const Vector3 b);

double
magnitude(Vector3 vector);

// Creates cube from initial conditions
Cube
create_cube(Vector3 __min, Vector3 __max, double __size);

#endif // GEOMETRY_H