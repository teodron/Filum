// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>



// TODO: reference additional headers your program requires here
#include "vector_math.h"
#include <iostream>
#include <vector>
#include <memory>
#include "FwdDecl.h"

typedef double Real;



using namespace std;
using namespace vmath;
const vec3<Real> zeroVec(0,0,0);
#include "PhysicsUtilities.h"