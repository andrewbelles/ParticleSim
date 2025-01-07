#include "../include/Geometry.h"

// Malloc Wrappers for single and double pntrs
void *
safe_malloc(size_t size)
{
  void *mem = malloc(size);
  if (mem == NULL) {
    fprintf(stderr, "Memory Alloc Failure\n");
    return NULL;
  }
  return mem;
}

// Boolean prime check
int
is_prime(const int num)
{
  if (num <= 1) return 0; // < 1 not prime
  if (num <= 3) return 1; // > 1 && <= 3 prime
  if (num % 2 == 0 || num % 3 == 0) return 0; // divisible by 2,3 not prime
  for (int i = 5; i * i <= num; i += 6) { // Since ^ checked go in 6k + 5 intervals
      if (num % i == 0 || num % (i + 2) == 0) return 0; // checks numbers not checked by ^
  }
  return 1; // returns prime if nothing hits
}

// Quick Inverrse Square Root
float
Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5;

	x2 = number * 0.5;
	y  = number;
	i  = * (long*) &y;						
	i  = 0x5f3759df - (i >> 1);               
	y  = * (float*) &i;
	y  = y * (threehalfs - ( x2 * y * y ));   

	return y;
}

// Normalizes using q_rsqrt
Vector3
Q_normalize(Vector3 a)
{
  double length = magnitude(a);
  return scaleVector(a, Q_rsqrt((float)length));
}

Vector3
normalize(Vector3 a)
{
  return scaleVector(a, 1.0 / sqrt(magnitude(a)));
}

// Returns a double between 0 and 10
double randomPosition() {
    return 6 * ((double)rand() / (double)RAND_MAX) + 2;
}

// Returns a vector of random values between -0.5 and 0.5 
Vector3
randomVector()
{
  return (Vector3){randomPosition(), randomPosition(), randomPosition()};
}

// Adds two vectors
Vector3
addVectors(Vector3 a, const Vector3 b)
{
  return (Vector3){a.x + b.x, a.y + b.y, a.z + b.z};
} 

// Adds scalar to vector components
Vector3
addScalar(const Vector3 a, const double scalar)
{
  return (Vector3){a.x + scalar, a.y + scalar, a.z + scalar};
}

// Subtracts two vectors
Vector3
subtractVectors(const Vector3 a, const Vector3 b)
{
  return (Vector3){a.x - b.x, a.y - b.y, a.z - b.z};
} 

// Subtracts a scalar from a vector components
Vector3
subtractScalar(const Vector3 a, const double scalar)
{
  return (Vector3){a.x - scalar, a.y - scalar, a.z - scalar};
}

// Scales vector by scalar
Vector3 
scaleVector(const Vector3 a, const double scalar)
{
  return (Vector3){a.x * scalar, a.y * scalar, a.z * scalar};
}

// Returns the cross product of two vectors
Vector3
crossProduct(Vector3 a, Vector3 b)
{
  return (Vector3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// Multiplies the transpose of a with b for dot Product
double
dotProduct(const Vector3 aT, const Vector3 b)
{
  return aT.x * b.x + aT.y * b.y + aT.z * b.z;
}

// Magnitude of a vector
double
magnitude(Vector3 a)
{
  return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

Cube
create_cube(Vector3 __min, Vector3 __max, double __size)
{
  Cube cube;
  cube.min = __min;
  cube.max = __max;
  cube.size = __size;
  return cube;
}