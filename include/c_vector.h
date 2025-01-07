#ifndef __C_VECTOR_H__
#define __C_VECTOR_H__ 

typedef struct {
  double data[3];
} vec3_double;

typedef struct {
  int data[3];
} vec3_int;

typedef struct {
  double *data;
  int count;
} vecflex_double;

typedef struct {
  int *data;
  int count; 
} vecflex_int;

// Function prototypes
void
initFlexibleVector_int(vecflex_int *vec, const int initial_count);

void
insertFlexibleVector_int(vecflex_int *vec, const int value);

void
initFlexibleVector_double(vecflex_int *vec, const int initial_count);

void
insertFlexibleVector_double(vecflex_int *vec, const int value);

vec3_double
addVec3_double(vec3_double a, vec3_double b);

vec3_double
subtractVec3_double(vec3_double a, vec3_double b);

vec3_double
scaleVec3_double(vec3_double a, double scalar);

double
magnitudeVec3_double(vec3_double a);

#endif // __C_VECTOR_H__