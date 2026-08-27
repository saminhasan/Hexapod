#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "pm_default_allocator.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ssc_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "Hexapod_dev_ceeedbd7_1_geometries.h"
PmfMessageId Hexapod_dev_ceeedbd7_1_checkDynamics ( const
RuntimeDerivedValuesBundle * rtdv , const double * state , const double *
input , const double * inputDot , const double * inputDdot , const double *
discreteState , const int * modeVector , int * errorResult ,
NeuDiagnosticManager * neDiagMgr ) { const double * rtdvd = rtdv -> mDoubles
. mValues ; const int * rtdvi = rtdv -> mInts . mValues ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) state ; ( void ) input ; ( void ) inputDot ; ( void ) inputDdot ; ( void ) discreteState ; ( void ) modeVector ; ( void ) neDiagMgr ; errorResult [ 0 ] = 0 ; return NULL ; }
