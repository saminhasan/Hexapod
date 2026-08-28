#ifndef Hexapod_dev_types_h_
#define Hexapod_dev_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_struct_2zFvE4klqMEEpiiM95wYGC_
#define DEFINED_TYPEDEF_FOR_struct_2zFvE4klqMEEpiiM95wYGC_
typedef struct { real_T Angle ; real_T Axis [ 3 ] ; }
struct_2zFvE4klqMEEpiiM95wYGC ;
#endif
#ifndef DEFINED_TYPEDEF_FOR_struct_NjJb0BLBoxXOwHw15Vx6hC_
#define DEFINED_TYPEDEF_FOR_struct_NjJb0BLBoxXOwHw15Vx6hC_
typedef struct { struct_2zFvE4klqMEEpiiM95wYGC Pos ; }
struct_NjJb0BLBoxXOwHw15Vx6hC ;
#endif
#ifndef DEFINED_TYPEDEF_FOR_struct_OdmtFjiCR5QAIiBTFdK6YF_
#define DEFINED_TYPEDEF_FOR_struct_OdmtFjiCR5QAIiBTFdK6YF_
typedef struct { struct_NjJb0BLBoxXOwHw15Vx6hC S ; void * ID ; }
struct_OdmtFjiCR5QAIiBTFdK6YF ;
#endif
#ifndef DEFINED_TYPEDEF_FOR_struct_PdEm5aibBpUO2JM4kXLMUF_
#define DEFINED_TYPEDEF_FOR_struct_PdEm5aibBpUO2JM4kXLMUF_
typedef struct { real_T mass ; real_T CoM [ 3 ] ; real_T MoI [ 3 ] ; real_T
PoI [ 3 ] ; real_T color [ 3 ] ; real_T opacity ; void * ID ; }
struct_PdEm5aibBpUO2JM4kXLMUF ;
#endif
#ifndef DEFINED_TYPEDEF_FOR_struct_djHv1IDeD06XoprYU3B4T_
#define DEFINED_TYPEDEF_FOR_struct_djHv1IDeD06XoprYU3B4T_
typedef struct { real_T translation [ 3 ] ; real_T angle ; real_T axis [ 3 ]
; void * ID ; } struct_djHv1IDeD06XoprYU3B4T ;
#endif
#ifndef DEFINED_TYPEDEF_FOR_struct_j64StuZ2rTU6bLJIiNazJE_
#define DEFINED_TYPEDEF_FOR_struct_j64StuZ2rTU6bLJIiNazJE_
typedef struct { real_T Pos ; } struct_j64StuZ2rTU6bLJIiNazJE ;
#endif
#ifndef DEFINED_TYPEDEF_FOR_struct_yi3YsCL79DNI0dtXLQLW7F_
#define DEFINED_TYPEDEF_FOR_struct_yi3YsCL79DNI0dtXLQLW7F_
typedef struct { struct_j64StuZ2rTU6bLJIiNazJE Rz ; void * ID ; }
struct_yi3YsCL79DNI0dtXLQLW7F ;
#endif
#ifndef DEFINED_TYPEDEF_FOR_struct_gih9egjPD1GTXsGE36a93E_
#define DEFINED_TYPEDEF_FOR_struct_gih9egjPD1GTXsGE36a93E_
typedef struct { struct_djHv1IDeD06XoprYU3B4T RigidTransform [ 284 ] ;
struct_PdEm5aibBpUO2JM4kXLMUF Solid [ 26 ] ; struct_yi3YsCL79DNI0dtXLQLW7F
RevoluteJoint [ 6 ] ; struct_OdmtFjiCR5QAIiBTFdK6YF SphericalJoint [ 12 ] ; }
struct_gih9egjPD1GTXsGE36a93E ;
#endif
#ifndef SS_UINT64
#define SS_UINT64 27
#endif
#ifndef SS_INT64
#define SS_INT64 28
#endif
typedef struct P_ P ;
#endif
