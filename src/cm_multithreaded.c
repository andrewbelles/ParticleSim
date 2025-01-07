#include "../include/cm_multithreaded.h"

/**** Initializes an array of objects with random state vectors ****/
Object *
InitializeObjects(const int count, const double radius)
{
  srand((unsigned int)time(NULL));
  Object *objects = (Object*)safe_malloc(count * sizeof(Object));

  // Initialization Loop 
  for (int i = 0; i < count; ++i) {
    objects[i].position = randomVector();
    objects[i].velocity = randomVector();
    objects[i].acceleration = (Vector3){0.0, 0.0, 0.0};
    objects[i].mass = 0.5;
    objects[i].radius = radius;
    objects[i].wall = (Int3){0, 0, 0};
  }
  return objects;
}

Vector3
Gravity(const Vector3 position, const Vector3 velocity)
{
  double ax = 0.0, ay = 0.0, az = 0.0;
  ay = -9.81;
  ax = 0.0;
  az = 0.0;

  return (Vector3){ax, ay, az};
}

Vector3
Hyperbolic(const Vector3 position, const Vector3 velocity)
{
  // System parameters
  double ax = 0.0, ay = 0.0, az = 0.0;

  // acceleration in z axis follows a Hyperbolic parabaloid  
  az = (position.x * position.y) / 2.0 - (velocity.x * velocity.y) / 2.0; 

  // Arbitrary trigonometric forces
  ax = sin(position.x) - cos(velocity.y);
  ay = cos(velocity.x) - sin(position.y);

  return (Vector3){ax, ay, az};         // Return completed acceleration vector
}

/**** Function passed through to thread to handle force update ****/
void *
ForceHelper(void *args)
{
  // Set Arguments passed by thread
  ForceArgs *force_args = (ForceArgs*)args;
  Object *objects = force_args->objects;
  const double dt = force_args->dt;
  const int start = force_args->start;
  const int count = force_args->count;
  PhysicsFunc physics = force_args->physics;

  for (int i = start; i < count; ++i) {
    // Half velocity
    objects[i].velocity = addVectors(objects[i].velocity, scaleVector(objects[i].acceleration, 0.5 * dt));
    
    // Next position
    objects[i].position = addVectors(objects[i].position, scaleVector(objects[i].velocity, dt));

    // Next Acceleration
    objects[i].acceleration = physics(objects[i].position, objects[i].velocity);

    // Next Velocity
    objects[i].velocity = addVectors(objects[i].velocity, scaleVector(objects[i].acceleration, 0.5 * dt));
  }

  return NULL;
}

/**** Velocity Verlet Integration for position update; Multithreaded ****/
void
ApplyForce(Object objects[], const double dt, const double count, PhysicsFunc physics)
{
  int thread_count = ((int)(count / 8) < 8) ? (int)(count / 8) : 8;
  pthread_t *threads = (pthread_t*)safe_malloc(thread_count * sizeof(pthread_t));
  ForceArgs *force_args = (ForceArgs*)safe_malloc(thread_count * sizeof(ForceArgs));

  int objects_per_thread = (int)(count / thread_count), remainder = (int)(count) % thread_count;

  for (int thread = 0; thread < thread_count; ++thread)  {
    // Set arguments for each thread
    force_args[thread].objects = objects;
    force_args[thread].dt = dt;
    force_args[thread].start = objects_per_thread * thread;
    force_args[thread].count = (thread == thread_count - 1) ? objects_per_thread * (thread + 1) + remainder : objects_per_thread * (thread + 1); 
    force_args[thread].physics = physics;

    // Call individual thread
    pthread_create(&threads[thread], NULL, ForceHelper, &force_args[thread]);
  }

  // Conjoin threads
  for (int thread = 0; thread < thread_count; ++thread) {
    pthread_join(threads[thread], NULL);
  }

  // Free memory 
  free(threads);
  free(force_args);
}

/**** Creates a 1D index from a 3D index input */
int
IndexCalc(const Int3 index, const int axis_ct)
{
  return index.x * axis_ct * axis_ct + index.y * axis_ct + index.z;
}

/**** Decomposes 1D index into a 3D index ****/
Int3
DecomposeIndex(const int index, const int axis_ct)
{
  return (Int3){(int)(index / (axis_ct * axis_ct)), (int)(index / (axis_ct)) % axis_ct, index % axis_ct};
}

/**** Add particle's absolute position to hashtable ****/
int
InsertObj(Map *map[], const int grid_index, const int obj_index)
{
  // Create memory
  Map *new_map = (Map*)safe_malloc(sizeof(Map));
  if (new_map == NULL) return -1;

  // Instantiate new_map map node
  new_map->obj_index = obj_index;
  new_map->next = NULL;

  // Insert into map
  if (map[grid_index] == NULL) {             // Empty Bucket
    new_map->count = 1;
    map[grid_index] = new_map;
  } else {
    new_map->next = map[grid_index];
    map[grid_index] = new_map;
    (map[grid_index]->count)++;
  }

  return 0;
}

/**** Map instantiation ****/
Map **
CreateMap(Object objects[], const Cube cube, 
          const int axis_ct, const int particle_ct, int *status)
{
  double partition_length = (double)(cube.size / axis_ct);
  Vector3 index_vec;
  int i = 0, partition_ct = axis_ct * axis_ct * axis_ct, grid_index = 0;
  Map **map = (Map**)safe_malloc(partition_ct * sizeof(Map*));
  Map *init;

  // Reset every bucket
  for (i = 0; i < partition_ct; i++) {
    init = (Map*)safe_malloc(sizeof(Map));
    init->next = NULL;
    init->count = 0;
    init->obj_index = -1;
    map[i] = init;
  }

  // Iterate and add each objects absolute position to the map TODO: MAKE FUNC
  for (i = 0; i < particle_ct; i++) {
    // Find index of absolute position

    index_vec = scaleVector(objects[i].position, 1.0 / partition_length);
    grid_index = IndexCalc((Int3){(int)index_vec.x, (int)index_vec.y, (int)index_vec.z}, axis_ct);

    // Place new particle in table
    if (InsertObj(map, grid_index, i) != 0) {
      DestroyMap(map, partition_ct);
      (*status) = -1; 
      return NULL;
    }
  }

  return map;
}

/**** Calculates how much the object's position needs to be shifted in a direction ****/
void
Overlap(double min, double max, double *position, double radius)
{
  double upper = max - (*position) - radius;
  double lower = (*position) - min - radius;

  if (upper < tol) {
    upper = fabs(upper) + tol;
    (*position) -= (upper);
  } else if (lower < tol) {
    lower = fabs(lower) + tol;
    (*position) += (lower);
  }
}

/**** Checks indices for out of bounds. Rectifies ****/
void
ProcessWall(Cube cube, Object objects[], const Int3 bad_index, const int obj_index, const int axis_ct)
{
  const double restitution = 1;      // Inelastic collision

  Vector3 min = cube.min, max = cube.max;

  // If the index is outside the proper bounds then we can assume that the object may have collided with the wall
  if ((bad_index.x >= axis_ct || bad_index.x < 0) && objects[obj_index].wall.x == 0) {
    // Computation to determine a collision with the wall
    if (max.x - objects[obj_index].position.x > (objects[obj_index].radius) && objects[obj_index].position.x - min.x > (objects[obj_index].radius)) {
      return;   // Return if it isn't colliding with the wall
    }

    // Reverse the direction of the velocity in the offending direction
    objects[obj_index].velocity.x *= -restitution;
    // Restore the position to a valid one
    Overlap(min.x, max.x, &objects[obj_index].position.x, objects[obj_index].radius);
    // For this specific iteration the object cannot interact with a wall again
    objects[obj_index].wall.x = 1;
  }
  
  // Functionally identical to above
  if ((bad_index.y >= axis_ct || bad_index.y < 0) && objects[obj_index].wall.y == 0) {
    if (max.y - objects[obj_index].position.y > (objects[obj_index].radius) && objects[obj_index].position.y - min.y > (objects[obj_index].radius)) {
      return;
    }

    objects[obj_index].velocity.y *= -restitution;
    Overlap(min.y, max.y, &objects[obj_index].position.y, objects[obj_index].radius);
    objects[obj_index].wall.y = 1;
  }

  if ((bad_index.z >= axis_ct || bad_index.z < 0) && objects[obj_index].wall.z == 0) {
    if (max.z - objects[obj_index].position.z > (objects[obj_index].radius) && objects[obj_index].position.z - min.z > (objects[obj_index].radius)) {
      return;
    }

    objects[obj_index].velocity.z *= -restitution;
    Overlap(min.z, max.z, &objects[obj_index].position.z, objects[obj_index].radius);
    objects[obj_index].wall.z = 1;
  }
}

/**** Handles a collision between two particles moving them along the axis of intersection*/
void
HandleCollision(Object *src, Object *deflecting)
{
  double Overlap;
  Vector3 src_new, deflecting_new;
  Vector3 relative_position, normal;

  // Find new velocities from conservation of momentum
  src_new = addVectors(scaleVector(src->velocity, (src->mass - deflecting->mass) / (src->mass + deflecting->mass)), 
                       scaleVector(deflecting->velocity, (2.0 * deflecting->mass) / (src->mass + deflecting->mass)));
  deflecting_new = addVectors(scaleVector(src->velocity, (2.0 * src->mass) / (src->mass + deflecting->mass)), 
                              scaleVector(deflecting->velocity, (src->mass - deflecting->mass) / (src->mass + deflecting->mass)));
  
  // Set new velocities
  src->velocity = src_new;
  deflecting->velocity = deflecting_new;

  // Correcting overlapped positions of particle 
  relative_position = subtractVectors(src->position, deflecting->position);
  Overlap = (src->radius + deflecting->radius) - magnitude(relative_position);
  normal = normalize(subtractVectors(deflecting->position, src->position));
  
  // Update positions based on overlap
  src->position = subtractVectors(src->position, scaleVector(normal, Overlap * 0.5));
  deflecting->position = addVectors(deflecting->position, scaleVector(normal, Overlap * 0.5));
}

/**** Calculates the scan array of indices for iterating through positions ****/
int *
CalculateIterator(Int3 (*scan)[3][3], Int3 center, const int axis_ct)
{
  
  int *indices = (int*)safe_malloc(27 * sizeof(int));

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        scan[k][j][i] = (Int3){center.x + (1 - k), center.y + (1 - j), center.z + (1 - i)};
        indices[k * 3 * 3 + j * 3 + i] = IndexCalc(scan[k][j][i], axis_ct);
      }
    }
  }

  return indices;
}

/**** Static check for if an index exceeds what is allowed in a direction */
static int
CheckIndex(const Int3 bad_index, const int axis_ct)
{
  if (bad_index.x >= axis_ct || bad_index.x < 0) {
    return 1;
  } else if (bad_index.y >= axis_ct || bad_index.y < 0) {
    return 1;
  } else if (bad_index.z >= axis_ct || bad_index.z < 0) {
    return 1;
  }
  return 0;
}

void *
CollisionHelper(void *args)
{
  // Pull arguments from thread input
  CollisionArgs *collision_args = (CollisionArgs*)args;
  Map **map = collision_args->map;
  Cube cube = collision_args->cube;
  Object *objects = collision_args->objects;
  int axis_count = collision_args->axis_count;

  // Local variable initialization 
  Map *current = NULL, *relative = NULL;
  Int3 collision_scan[3][3][3], center, bad_index;
  int *indices = NULL, obj_index = -1; 
  double distance = 0.0;

  // Iterates from start to end as specified by args passed through thread
  for (int partition = 0; partition < 64; ++partition) {

    // Skip if partition is empty
    if (map[partition]->count == 0) continue;

    // Decompose center and surrounding 3x3x3
    center = DecomposeIndex(partition, axis_count);
    indices = CalculateIterator(collision_scan, center, axis_count);

    // Set current Map pointer
    current = map[partition];
    while (current != NULL) {
      obj_index = current->obj_index;

      for (int index = 0; index < 27; ++index) {

        bad_index = DecomposeIndex(index, 3);
        bad_index = collision_scan[bad_index.x][bad_index.y][bad_index.z];

        if (CheckIndex(bad_index, axis_count) == 1) {
          // Process check for wall collision
          (void)ProcessWall(cube, objects, bad_index, obj_index, axis_count);
          continue;               // Can't collide with something out of bounds
        }

        // If the bucket is empty skip
        if (map[indices[index]] == NULL) continue;

        // Set the relative bucket
        relative = map[indices[index]];
        while (relative != NULL) {

          // If the object index of the relative matches the current index then skip
          if (relative->obj_index != current->obj_index) {

            // If the distance is less than sum of radii then there is a collision
            distance = magnitude(subtractVectors(objects[obj_index].position, objects[relative->obj_index].position));
            if (distance < (objects[obj_index].radius + objects[relative->obj_index].radius)) {
              (void)HandleCollision(&objects[obj_index], &objects[relative->obj_index]);
            }

          }
          // If bucket is empty after then break else advance
          if (relative->next == NULL) break;
          relative = relative->next;
        }
      }
      // Same concept as above but for current map pointer
      if (current->next == NULL) break;
      current = current->next;
    }
    free(indices);                // Free the indices for this iteration
  }

  return NULL;
}

/**** Multi-threaded function to handle all collisions for a sub step ****/
void
CollisionCall(Cube cube, Object objects[], 
              const int partition_ct, const int particle_ct, const int axis_ct)
{
  int status = 0;
  Map **map = CreateMap(objects, cube, axis_ct, particle_ct, &status);
  int thread_count = 8, map_index = 0, index = 0;
  Int3 sub_index;
  Map ***thread_map = (Map***)safe_malloc(thread_count * sizeof(Map**));
  pthread_t *threads = (pthread_t*)safe_malloc(thread_count * sizeof(pthread_t));
  pthread_barrier_t barrier;
  CollisionArgs *collision_args = (CollisionArgs*)safe_malloc(thread_count * sizeof(CollisionArgs));

  pthread_barrier_init(&barrier, NULL, thread_count);

  for (int thread = 0; thread < thread_count; ++thread) {

    // Set wall integer vector to 0
    for (int particle = 0; particle < particle_ct; ++particle) {
      objects[particle].wall = (Int3){0, 0, 0};
    }

    // Create sub map specific for thread 
    thread_map[thread] = (Map**)safe_malloc(64 * sizeof(Map*));

    // Update sub index depending on thread number
    sub_index.z = (thread % 2) ? 4 : 0;
    sub_index.y = ((thread >> 1) & 1) ? 4 : 0;
    sub_index.x = ((thread >> 2) & 1) ? 4 : 0;

    // Fill thread map which will be the effect sub map for each map sent to thread
    index = 0;
    for (int x = sub_index.z; x < sub_index.z + 4; ++x) {
      for (int y = sub_index.y; y < sub_index.y + 4; ++y) {
        for (int z = sub_index.x; z < sub_index.x + 4; ++z) {
          map_index = IndexCalc((Int3){x, y, z}, 8);
          thread_map[thread][index++] = map[map_index]; 
        }
      }
    }


    // Set all arguments for current thread
    collision_args[thread].cube = cube;
    collision_args[thread].objects = objects;
    collision_args[thread].map = thread_map[thread];
    collision_args[thread].axis_count = 4;
    collision_args[thread].barrier = &barrier;

    // Create individual thread 
    pthread_create(&threads[thread], NULL, CollisionHelper, &collision_args[thread]);
  }

  // Conjoin threads 
  for (int thread = 0; thread < thread_count; ++thread) {
    pthread_join(threads[thread], NULL);
    free(thread_map[thread]);
  }

  // Free memory allocated to map
  free(threads);
  free(collision_args);
  free(thread_map);
  DestroyMap(map, partition_ct); 
  pthread_barrier_destroy(&barrier);
}

/**** Frees memory allocated to a map structure */
void
DestroyMap(Map *map[], const int size)
{
  Map *curr, *destroy;
  // Iterates through array portion
  for (int i = 0; i < size; i++) {
    curr = map[i];
    // Iterates through l_list portion and frees each node
    while (curr != NULL) {
      destroy = curr;
      curr = curr->next;
      free(destroy);
    }
  }
  free(map);
}

/**** Prints all particles positions ****/
void
PrintPositions(const Object objects[], const int count)
{
  for (int i = 0; i < count; i++) {
    printf(">> Particle %d:\n", i + 1);
    printf("  Pos: <%.3lf,%.3lf,%.3lf>\n", objects[i].position.x, objects[i].position.y, objects[i].position.z);
    printf("  Vel: <%.3lf,%.3lf,%.3lf>\n", objects[i].velocity.x, objects[i].velocity.y, objects[i].velocity.z);
    printf("  Accel: <%.3lf,%.3lf,%.3lf>\n", objects[i].acceleration.x, objects[i].acceleration.y, objects[i].acceleration.z);
  }
}