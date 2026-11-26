#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

// The brain is created at the top of src/main.cpp, before TankComp.cpp is
// included (VEXcode does the same behind the scenes), so TankComp.cpp never
// declares "brain Brain;" itself.
extern vex::brain Brain;


#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)