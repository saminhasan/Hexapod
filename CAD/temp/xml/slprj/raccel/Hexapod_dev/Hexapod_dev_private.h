#ifndef Hexapod_dev_private_h_
#define Hexapod_dev_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include <stddef.h>
#include "rtsplntypes.h"
#include "Hexapod_dev_types.h"
#include "Hexapod_dev.h"
#if !defined(rt_VALIDATE_MEMORY)
#define rt_VALIDATE_MEMORY(S, ptr)     if(!(ptr)) {\
    ssSetErrorStatus(rtS, RT_MEMORY_ALLOCATION_ERROR);\
    }
#endif
#if !defined(rt_FREE)
#if !defined(_WIN32)
#define rt_FREE(ptr)     if((ptr) != (NULL)) {\
    free((ptr));\
    (ptr) = (NULL);\
    }
#else
#define rt_FREE(ptr)     if((ptr) != (NULL)) {\
    free((void *)(ptr));\
    (ptr) = (NULL);\
    }
#endif
#endif
real_T look_SplNBinXZcd ( uint32_T numDims , const real_T * u , const
rt_LUTSplineWork * const SWork ) ; void rt_Spline2Derivd ( const real_T * x ,
const real_T * y , uint32_T n , real_T * u , real_T * y2 ) ; real_T
intrp_NSplcd ( uint32_T numDims , const rt_LUTSplineWork * const splWork ,
uint32_T extrapMethod ) ; extern uint32_T plook_binx ( real_T u , const
real_T bp [ ] , uint32_T maxIndex , real_T * fraction ) ; extern uint32_T
binsearch_u32d ( real_T u , const real_T bp [ ] , uint32_T startIndex ,
uint32_T maxIndex ) ;
#if defined(MULTITASKING)
#error Model (Hexapod_dev) was built in \SingleTasking solver mode, however the MULTITASKING define is \present. If you have multitasking (e.g. -DMT or -DMULTITASKING) \defined on the Code Generation page of Simulation parameter dialog, please \remove it and on the Solver page, select solver mode \MultiTasking. If the Simulation parameter dialog is configured \correctly, please verify that your template makefile is \configured correctly.
#endif
#endif
