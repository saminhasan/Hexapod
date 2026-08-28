#ifndef Hexapod_dev_h_
#define Hexapod_dev_h_
#include <cmath>
#include <stdlib.h>
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "rtwtypes.h"
#include "sigstream_rtw.h"
#include "simtarget/slSimTgtSigstreamRTW.h"
#include "simtarget/slSimTgtSlioCoreRTW.h"
#include "simtarget/slSimTgtSlioClientsRTW.h"
#include "simtarget/slSimTgtSlioSdiRTW.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "raccel.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "rt_logging_simtarget.h"
#include "rt_nonfinite.h"
#include "dt_info.h"
#include "ext_work.h"
#include "nesl_rtw.h"
#include "Hexapod_dev_ceeedbd7_1_gateway.h"
#include "ssc_rtw_logging.h"
#include "physmod/common/logging2/core/rtw/rtw_log_fcn_manager.h"
#include "stdlib.h"
#include "physmod/common/logging2/core/rtw/SscRTWLogging.h"
#include "Hexapod_dev_types.h"
#include <stddef.h>
#include "rtsplntypes.h"
extern "C" {
#include "rtGetInf.h"
}
#include "rtw_modelmap.h"
#include "rtw_modelmap_simtarget.h"
#include "rt_defines.h"
#include <string.h>
#define MODEL_NAME Hexapod_dev
#define NSAMPLE_TIMES (3) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (14) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (61)   
#elif NCSTATES != 61
#error Invalid specification of NCSTATES defined in compiler command
#endif
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
#endif
#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
#endif
#ifndef IN_RACCEL_MAIN
#endif
typedef struct { real_T aslpipoci2 ; real_T fzxmotide1 ; real_T kfjnjnbiln ;
real_T bkamu4msu4 ; real_T ecpp33hi3p ; real_T gtyqmeteln ; real_T gmkqayjiuv
[ 4 ] ; real_T du1olrgvrn [ 4 ] ; real_T mcnp4idm3j [ 4 ] ; real_T fns5dbtm53
[ 4 ] ; real_T fq1dlz2two [ 4 ] ; real_T lssf54mn5z [ 4 ] ; real_T lgfnqwstci
[ 49 ] ; real_T i4kzipy3mq [ 19 ] ; } B ; typedef struct { real_T o35huj2gsd
; real_T klzhkf4szf ; real_T dagi3ssnao ; real_T eeo3dalye1 ; real_T
idl3np2sad ; real_T a0lmv0bgi0 ; real_T fmzddgcets ; real_T blbm45o2uq ;
real_T mbr2ermefj ; real_T dnv5nxxb53 ; real_T hklg4jlt4z ; real_T mjt4xoczh4
; real_T ev2m2wgy2j ; real_T fg5gnsupey ; real_T nopt2maj0d ; real_T
nukyf0pyaa ; real_T comai21xuo [ 10001 ] ; real_T batgpdl5hb [ 10001 ] ;
real_T darmnwyzix ; real_T plfvcgjov5 ; real_T i4pwvk5fud ; real_T empqd1cr2q
; real_T jge2c5avbd [ 10001 ] ; real_T k1dqwarsjy [ 10001 ] ; real_T
cp3e04jgmd ; real_T i4rnfp1wv3 ; real_T dpbxldkwzj ; real_T e2ooci3cok ;
real_T bciralxndx [ 10001 ] ; real_T j0h5r2rwwx [ 10001 ] ; real_T dcbjunmnuz
; real_T d0piouqmcg ; real_T b3exesx2ov ; real_T k4irirzhzl ; real_T
nadia4sijt [ 10001 ] ; real_T btlye4fecs [ 10001 ] ; real_T ap0qucs3ix ;
real_T clnubtij1c ; real_T ga4gvlvb3k ; real_T fdnrequow3 ; real_T esd12dsf3p
[ 10001 ] ; real_T lw1aa3ikun [ 10001 ] ; real_T hig0ipabbz ; real_T
d2a3xixuf0 ; real_T nsivhr4agq ; real_T hiwcepl5bw ; real_T dm5bb5xb02 [
10001 ] ; real_T a4gdm1csbd [ 10001 ] ; real_T hv225z33ok ; real_T c0mmrfri1b
; real_T ejjqivakel ; real_T duhp5c0dw3 ; real_T mzkea3zvvh ; real_T
orrqp05dyy ; void * jfocsgvjr2 ; void * etlkslxoll [ 6 ] ; void * lcknyo52qu
[ 9 ] ; void * bwgtinpdi5 ; void * glqjd0j2v1 [ 6 ] ; void * dqohor0ufz [ 9 ]
; void * iha3lbqkcg ; void * fee4e2lshm [ 6 ] ; void * k4afwk3pmb [ 9 ] ;
void * cayhrhdjj2 ; void * eh4jhgdpjq [ 6 ] ; void * nlcxo5f5f4 [ 9 ] ; void
* fza5104ubd ; void * bot0f4jvu3 [ 6 ] ; void * nvuzcubtym [ 9 ] ; void *
og4xbrvaxl ; void * gsuie3xmsc [ 6 ] ; void * kgv4ez4phl [ 9 ] ; void *
ngbzoo1n2w ; void * cn01mlmqpq ; void * dzz3fjwjh4 ; void * pmulv1s4uc ; void
* dqtv4j00nz ; void * lyxq4lljok ; void * a3motogwve ; void * o0r4inb335 ;
void * edpwaq1mh5 ; void * i523mfvkwd ; struct { void * AQHandles ; }
a0jpeenedk ; struct { void * AQHandles ; } hpsgqxjk3e ; struct { void *
AQHandles ; } iaizpaoviz ; struct { void * AQHandles ; } agizt5nhn4 ; struct
{ void * AQHandles ; } f54c0stnai ; struct { void * AQHandles ; } fhypfegxnq
; void * m1elosxkow ; void * lhzhdx15na ; void * ljxcsqwcit ; void *
h4jnxgasty ; void * asduixo1nh ; int32_T fusdopd4md ; int32_T lbojcaqzep ;
int32_T ozm0zgbvmf ; int32_T jdsiaq5mqq ; int32_T gyvass5xmg ; int32_T
hcjb1vjnr1 ; uint32_T bmm15pnlh2 ; uint32_T lu02c5leuy ; uint32_T chb23a0xlk
; uint32_T cheuqjrosp ; uint32_T bbm34zsyfk ; uint32_T juxxtpbgcs ; int_T
ee11ecv44v ; int_T cu4ltknb50 ; int_T arrnm4nhoz ; int_T mskbr31frt ; uint8_T
irnshlasoh ; uint8_T k3w4ik3bnp ; uint8_T az4qyz5ina ; uint8_T d5ypu3fzep ;
uint8_T asbkawbx5i ; uint8_T pfmvz0ju2f ; uint8_T jjr0g32epl ; uint8_T
m1v51w4plc ; uint8_T bwaatfgl13 ; uint8_T ghbw0wjs25 ; boolean_T bdszturt0n ;
boolean_T mngswn4g4e ; } DW ; typedef struct { real_T ffpzz1mj1h [ 2 ] ;
real_T jkkzntry25 [ 2 ] ; real_T do5rtgbssy [ 2 ] ; real_T jvqsyd2juc [ 2 ] ;
real_T hbonywumxq [ 2 ] ; real_T my4o33ecsh [ 2 ] ; real_T dr1jyp5m1z [ 49 ]
; } X ; typedef struct { real_T ffpzz1mj1h [ 2 ] ; real_T jkkzntry25 [ 2 ] ;
real_T do5rtgbssy [ 2 ] ; real_T jvqsyd2juc [ 2 ] ; real_T hbonywumxq [ 2 ] ;
real_T my4o33ecsh [ 2 ] ; real_T dr1jyp5m1z [ 49 ] ; } XDot ; typedef struct
{ boolean_T ffpzz1mj1h [ 2 ] ; boolean_T jkkzntry25 [ 2 ] ; boolean_T
do5rtgbssy [ 2 ] ; boolean_T jvqsyd2juc [ 2 ] ; boolean_T hbonywumxq [ 2 ] ;
boolean_T my4o33ecsh [ 2 ] ; boolean_T dr1jyp5m1z [ 49 ] ; } XDis ; typedef
struct { rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct P_ { real_T
time [ 10001 ] ; real_T uDLookupTable_tableData [ 10001 ] ; real_T Gain_Gain
; real_T uDLookupTable_tableData_ngisugqubh [ 10001 ] ; real_T Gain1_Gain ;
real_T uDLookupTable_tableData_ai1chejlcf [ 10001 ] ; real_T
Gain_Gain_mphseqoivg ; real_T uDLookupTable_tableData_l2sller51b [ 10001 ] ;
real_T Gain_Gain_cmu3wv0ql2 ; real_T uDLookupTable_tableData_osrraa44zy [
10001 ] ; real_T Gain_Gain_fhgg2tbdth ; real_T
uDLookupTable_tableData_nuebhjad2j [ 10001 ] ; real_T Gain_Gain_g31zf1epte ;
real_T Constant_Value ; real_T Constant_Value_bjkzcm1v50 ; real_T
Constant_Value_cuijmfc0gq ; real_T Constant_Value_h3mrzjkbom ; real_T
Constant_Value_heuixq11n0 ; real_T Constant_Value_jvff2kcwkv ; uint32_T
uDLookupTable_maxIndex ; uint32_T uDLookupTable_dimSizes ; uint32_T
uDLookupTable_numYWorkElts [ 2 ] ; uint32_T uDLookupTable_maxIndex_gecwmd542g
; uint32_T uDLookupTable_dimSizes_dplxybs3xu ; uint32_T
uDLookupTable_numYWorkElts_pvvzvokkjh [ 2 ] ; uint32_T
uDLookupTable_maxIndex_kmnxkox2xs ; uint32_T
uDLookupTable_dimSizes_gpxa1msfef ; uint32_T
uDLookupTable_numYWorkElts_bgs0ock0jg [ 2 ] ; uint32_T
uDLookupTable_maxIndex_ewjlqfurjs ; uint32_T
uDLookupTable_dimSizes_jln4ytncld ; uint32_T
uDLookupTable_numYWorkElts_pjunzd1vya [ 2 ] ; uint32_T
uDLookupTable_maxIndex_imwny5u4rw ; uint32_T
uDLookupTable_dimSizes_fvopwqzbwe ; uint32_T
uDLookupTable_numYWorkElts_iiusdsomba [ 2 ] ; uint32_T
uDLookupTable_maxIndex_o1qqvqqib2 ; uint32_T
uDLookupTable_dimSizes_nyynambagj ; uint32_T
uDLookupTable_numYWorkElts_on1gheggyj [ 2 ] ; } ;
#ifdef __cplusplus
extern "C" {
#endif
extern const char_T * RT_MEMORY_ALLOCATION_ERROR ;
#ifdef __cplusplus
}
#endif
extern B rtB ; extern X rtX ; extern DW rtDW ; extern P rtP ; extern mxArray
* mr_Hexapod_dev_GetDWork ( ) ; extern void mr_Hexapod_dev_SetDWork ( const
mxArray * ssDW ) ; extern mxArray *
mr_Hexapod_dev_GetSimStateDisallowedBlocks ( ) ;
#ifdef __cplusplus 
extern "C" {
#endif
#ifdef __cplusplus 
}
#endif
#ifdef __cplusplus 
extern "C" {
#endif
#ifdef __cplusplus 
}
#endif
extern const rtwCAPI_ModelMappingStaticInfo * Hexapod_dev_GetCAPIStaticMap ( void ) ;
#ifdef __cplusplus
extern "C" {
#endif
extern SimStruct * const rtS ;
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
extern "C" {
#endif
extern DataMapInfo * rt_dataMapInfoPtr ; extern rtwCAPI_ModelMappingInfo *
rt_modelMapInfoPtr ; void MdlOutputs ( int_T tid ) ; void
MdlOutputsParameterSampleTime ( int_T tid ) ; void MdlUpdate ( int_T tid ) ;
void MdlTerminate ( void ) ; void MdlInitializeSizes ( void ) ; void
MdlInitializeSampleTimes ( void ) ; SimStruct * raccel_register_model ( ssExecutionInfo * executionInfo ) ;
#ifdef __cplusplus
}
#endif
#endif
