// Only for VS Code's error checking (IntelliSense). VEXcode V5 supplies its own
// vex.h when it builds, so this file is never downloaded to the robot.
// It matches what VEXcode adds to a text project, including "brain Brain;".
#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "v5.h"
#include "v5_vcs.h"

extern vex::brain Brain;

#define waitUntil(condition) do { wait(5, msec); } while (!(condition))
#define repeat(iterations) for (int iterator = 0; iterator < iterations; iterator++)
