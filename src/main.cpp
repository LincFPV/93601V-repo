// VS Code VEX project entry point.
//
// The real program is TankComp.cpp in the main folder: edit that file, not this
// one. Keeping the code in one place means VEXcode (TankComp.v5cpp) and VS Code
// always build the same program. Run scripts/sync.sh to copy edits into the
// .v5cpp.
#include "vex.h"

// The brain must be created BEFORE anything in TankComp.cpp, because
// TankComp.cpp builds the claw from Brain.ThreeWirePort.A when the program
// starts. Objects in one file are created top to bottom, so defining the
// brain here, above the #include, guarantees that order. VEXcode does the same
// thing behind the scenes, which is why TankComp.cpp never declares it.
// (Putting it in a separate .cpp file crashed the brain with a memory
// permission error: the claw was created before the brain existed.)
vex::brain Brain;

#include "../TankComp.cpp"
