#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "Hexapod_dev_capi_host.h"
#define sizeof(...) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)
#ifndef SS_UINT64
#define SS_UINT64 27
#endif
#ifndef SS_INT64
#define SS_INT64 28
#endif
#else
#include "builtin_typeid_types.h"
#include "Hexapod_dev.h"
#include "Hexapod_dev_capi.h"
#include "Hexapod_dev_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((NULL))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 0 , TARGET_STRING ( "Hexapod_dev/Subsystem1/Add" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 1 , 0 , TARGET_STRING ( "Hexapod_dev/Subsystem2/Add" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 2 , 0 , TARGET_STRING ( "Hexapod_dev/Subsystem3/Add" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 3 , 0 , TARGET_STRING ( "Hexapod_dev/Subsystem4/Add" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 4 , 0 , TARGET_STRING ( "Hexapod_dev/Subsystem5/Add" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 5 , 0 , TARGET_STRING ( "Hexapod_dev/Subsystem6/Add" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 6 , 0 , TARGET_STRING ( "Hexapod_dev/Solver Configuration/EVAL_KEY/INPUT_1_1_1" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 7 , 0 , TARGET_STRING ( "Hexapod_dev/Solver Configuration/EVAL_KEY/INPUT_2_1_1" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 8 , 0 , TARGET_STRING ( "Hexapod_dev/Solver Configuration/EVAL_KEY/INPUT_3_1_1" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 9 , 0 , TARGET_STRING ( "Hexapod_dev/Solver Configuration/EVAL_KEY/INPUT_4_1_1" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 10 , 0 , TARGET_STRING ( "Hexapod_dev/Solver Configuration/EVAL_KEY/INPUT_5_1_1" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 11 , 0 , TARGET_STRING ( "Hexapod_dev/Solver Configuration/EVAL_KEY/INPUT_6_1_1" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 12 , 0 , TARGET_STRING ( "Hexapod_dev/Solver Configuration/EVAL_KEY/OUTPUT_1_0" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 13 , 0 , TARGET_STRING ( "Hexapod_dev/Solver Configuration/EVAL_KEY/STATE_1" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_BlockParameters rtBlockParameters [ ] = { { 14 , TARGET_STRING ( "Hexapod_dev/Subsystem1/Constant" ) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 15 , TARGET_STRING ( "Hexapod_dev/Subsystem1/Gain" ) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 16 , TARGET_STRING ( "Hexapod_dev/Subsystem1/1-D Lookup Table" ) , TARGET_STRING ( "Table" ) , 0 , 4 , 0 } , { 17 , TARGET_STRING ( "Hexapod_dev/Subsystem1/1-D Lookup Table" ) , TARGET_STRING ( "maxIndex" ) , 1 , 0 , 0 } , { 18 , TARGET_STRING ( "Hexapod_dev/Subsystem1/1-D Lookup Table" ) , TARGET_STRING ( "dimSizes" ) , 1 , 0 , 0 } , { 19 , TARGET_STRING ( "Hexapod_dev/Subsystem1/1-D Lookup Table" ) , TARGET_STRING ( "numYWorkElts" ) , 1 , 5 , 0 } , { 20 , TARGET_STRING ( "Hexapod_dev/Subsystem2/Constant" ) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 21 , TARGET_STRING ( "Hexapod_dev/Subsystem2/Gain1" ) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 22 , TARGET_STRING ( "Hexapod_dev/Subsystem2/1-D Lookup Table" ) , TARGET_STRING ( "Table" ) , 0 , 4 , 0 } , { 23 , TARGET_STRING ( "Hexapod_dev/Subsystem2/1-D Lookup Table" ) , TARGET_STRING ( "maxIndex" ) , 1 , 0 , 0 } , { 24 , TARGET_STRING ( "Hexapod_dev/Subsystem2/1-D Lookup Table" ) , TARGET_STRING ( "dimSizes" ) , 1 , 0 , 0 } , { 25 , TARGET_STRING ( "Hexapod_dev/Subsystem2/1-D Lookup Table" ) , TARGET_STRING ( "numYWorkElts" ) , 1 , 5 , 0 } , { 26 , TARGET_STRING ( "Hexapod_dev/Subsystem3/Constant" ) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 27 , TARGET_STRING ( "Hexapod_dev/Subsystem3/Gain" ) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 28 , TARGET_STRING ( "Hexapod_dev/Subsystem3/1-D Lookup Table" ) , TARGET_STRING ( "Table" ) , 0 , 4 , 0 } , { 29 , TARGET_STRING ( "Hexapod_dev/Subsystem3/1-D Lookup Table" ) , TARGET_STRING ( "maxIndex" ) , 1 , 0 , 0 } , { 30 , TARGET_STRING ( "Hexapod_dev/Subsystem3/1-D Lookup Table" ) , TARGET_STRING ( "dimSizes" ) , 1 , 0 , 0 } , { 31 , TARGET_STRING ( "Hexapod_dev/Subsystem3/1-D Lookup Table" ) , TARGET_STRING ( "numYWorkElts" ) , 1 , 5 , 0 } , { 32 , TARGET_STRING ( "Hexapod_dev/Subsystem4/Constant" ) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 33 , TARGET_STRING ( "Hexapod_dev/Subsystem4/Gain" ) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 34 , TARGET_STRING ( "Hexapod_dev/Subsystem4/1-D Lookup Table" ) , TARGET_STRING ( "Table" ) , 0 , 4 , 0 } , { 35 , TARGET_STRING ( "Hexapod_dev/Subsystem4/1-D Lookup Table" ) , TARGET_STRING ( "maxIndex" ) , 1 , 0 , 0 } , { 36 , TARGET_STRING ( "Hexapod_dev/Subsystem4/1-D Lookup Table" ) , TARGET_STRING ( "dimSizes" ) , 1 , 0 , 0 } , { 37 , TARGET_STRING ( "Hexapod_dev/Subsystem4/1-D Lookup Table" ) , TARGET_STRING ( "numYWorkElts" ) , 1 , 5 , 0 } , { 38 , TARGET_STRING ( "Hexapod_dev/Subsystem5/Constant" ) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 39 , TARGET_STRING ( "Hexapod_dev/Subsystem5/Gain" ) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 40 , TARGET_STRING ( "Hexapod_dev/Subsystem5/1-D Lookup Table" ) , TARGET_STRING ( "Table" ) , 0 , 4 , 0 } , { 41 , TARGET_STRING ( "Hexapod_dev/Subsystem5/1-D Lookup Table" ) , TARGET_STRING ( "maxIndex" ) , 1 , 0 , 0 } , { 42 , TARGET_STRING ( "Hexapod_dev/Subsystem5/1-D Lookup Table" ) , TARGET_STRING ( "dimSizes" ) , 1 , 0 , 0 } , { 43 , TARGET_STRING ( "Hexapod_dev/Subsystem5/1-D Lookup Table" ) , TARGET_STRING ( "numYWorkElts" ) , 1 , 5 , 0 } , { 44 , TARGET_STRING ( "Hexapod_dev/Subsystem6/Constant" ) , TARGET_STRING ( "Value" ) , 0 , 0 , 0 } , { 45 , TARGET_STRING ( "Hexapod_dev/Subsystem6/Gain" ) , TARGET_STRING ( "Gain" ) , 0 , 0 , 0 } , { 46 , TARGET_STRING ( "Hexapod_dev/Subsystem6/1-D Lookup Table" ) , TARGET_STRING ( "Table" ) , 0 , 4 , 0 } , { 47 , TARGET_STRING ( "Hexapod_dev/Subsystem6/1-D Lookup Table" ) , TARGET_STRING ( "maxIndex" ) , 1 , 0 , 0 } , { 48 , TARGET_STRING ( "Hexapod_dev/Subsystem6/1-D Lookup Table" ) , TARGET_STRING ( "dimSizes" ) , 1 , 0 , 0 } , { 49 , TARGET_STRING ( "Hexapod_dev/Subsystem6/1-D Lookup Table" ) , TARGET_STRING ( "numYWorkElts" ) , 1 , 5 , 0 } , { 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 } } ; static int_T rt_LoggedStateIdxList [ ] = { - 1 } ; static const rtwCAPI_Signals rtRootInputs [ ] = { { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_Signals rtRootOutputs [ ] = { { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_ModelParameters rtModelParameters [ ] = { { 50 , TARGET_STRING ( "time" ) , 0 , 4 , 0 } , { 0 , ( NULL ) , 0 , 0 , 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . aslpipoci2 , & rtB . fzxmotide1 ,
& rtB . kfjnjnbiln , & rtB . bkamu4msu4 , & rtB . ecpp33hi3p , & rtB .
gtyqmeteln , & rtB . gmkqayjiuv [ 0 ] , & rtB . du1olrgvrn [ 0 ] , & rtB .
fq1dlz2two [ 0 ] , & rtB . lssf54mn5z [ 0 ] , & rtB . mcnp4idm3j [ 0 ] , &
rtB . fns5dbtm53 [ 0 ] , & rtB . i4kzipy3mq [ 0 ] , & rtB . lgfnqwstci [ 0 ]
, & rtP . Constant_Value , & rtP . Gain_Gain , & rtP .
uDLookupTable_tableData [ 0 ] , & rtP . uDLookupTable_maxIndex , & rtP .
uDLookupTable_dimSizes , & rtP . uDLookupTable_numYWorkElts [ 0 ] , & rtP .
Constant_Value_bjkzcm1v50 , & rtP . Gain1_Gain , & rtP .
uDLookupTable_tableData_ngisugqubh [ 0 ] , & rtP .
uDLookupTable_maxIndex_gecwmd542g , & rtP . uDLookupTable_dimSizes_dplxybs3xu
, & rtP . uDLookupTable_numYWorkElts_pvvzvokkjh [ 0 ] , & rtP .
Constant_Value_cuijmfc0gq , & rtP . Gain_Gain_mphseqoivg , & rtP .
uDLookupTable_tableData_ai1chejlcf [ 0 ] , & rtP .
uDLookupTable_maxIndex_kmnxkox2xs , & rtP . uDLookupTable_dimSizes_gpxa1msfef
, & rtP . uDLookupTable_numYWorkElts_bgs0ock0jg [ 0 ] , & rtP .
Constant_Value_h3mrzjkbom , & rtP . Gain_Gain_cmu3wv0ql2 , & rtP .
uDLookupTable_tableData_l2sller51b [ 0 ] , & rtP .
uDLookupTable_maxIndex_ewjlqfurjs , & rtP . uDLookupTable_dimSizes_jln4ytncld
, & rtP . uDLookupTable_numYWorkElts_pjunzd1vya [ 0 ] , & rtP .
Constant_Value_heuixq11n0 , & rtP . Gain_Gain_fhgg2tbdth , & rtP .
uDLookupTable_tableData_osrraa44zy [ 0 ] , & rtP .
uDLookupTable_maxIndex_imwny5u4rw , & rtP . uDLookupTable_dimSizes_fvopwqzbwe
, & rtP . uDLookupTable_numYWorkElts_iiusdsomba [ 0 ] , & rtP .
Constant_Value_jvff2kcwkv , & rtP . Gain_Gain_g31zf1epte , & rtP .
uDLookupTable_tableData_nuebhjad2j [ 0 ] , & rtP .
uDLookupTable_maxIndex_o1qqvqqib2 , & rtP . uDLookupTable_dimSizes_nyynambagj
, & rtP . uDLookupTable_numYWorkElts_on1gheggyj [ 0 ] , & rtP . time [ 0 ] ,
} ; static int32_T * rtVarDimsAddrMap [ ] = { ( NULL ) } ;
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , ( uint8_T ) SS_DOUBLE , 0 , 0 , 0 } ,
{ "unsigned int" , "uint32_T" , 0 , 0 , sizeof ( uint32_T ) , ( uint8_T )
SS_UINT32 , 0 , 0 , 0 } } ;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static const rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_SCALAR , 0 , 2 , 0 } , { rtwCAPI_VECTOR , 2 , 2 , 0 } , {
rtwCAPI_VECTOR , 4 , 2 , 0 } , { rtwCAPI_VECTOR , 6 , 2 , 0 } , {
rtwCAPI_VECTOR , 8 , 2 , 0 } , { rtwCAPI_VECTOR , 10 , 2 , 0 } } ; static
const uint_T rtDimensionArray [ ] = { 1 , 1 , 4 , 1 , 19 , 1 , 49 , 1 , 10001
, 1 , 2 , 1 } ; static const real_T rtcapiStoredFloats [ ] = { 0.0 } ; static
const rtwCAPI_FixPtMap rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) ,
rtwCAPI_FIX_RESERVED , 0 , 0 , ( boolean_T ) 0 } , } ; static const
rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { static_cast < const void * >
( & rtcapiStoredFloats [ 0 ] ) , static_cast < const void * > ( &
rtcapiStoredFloats [ 0 ] ) , static_cast < int8_T > ( 0 ) , static_cast <
uint8_T > ( 0 ) } } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { {
rtBlockSignals , 14 , rtRootInputs , 0 , rtRootOutputs , 0 } , {
rtBlockParameters , 36 , rtModelParameters , 1 } , { ( NULL ) , 0 } , {
rtDataTypeMap , rtDimensionMap , rtFixPtMap , rtElementMap , rtSampleTimeMap
, rtDimensionArray } , "float" , { 2301247433U , 1297405112U , 3492532633U ,
2774462004U } , ( NULL ) , 0 , ( boolean_T ) 0 , rt_LoggedStateIdxList } ;
const rtwCAPI_ModelMappingStaticInfo * Hexapod_dev_GetCAPIStaticMap ( void )
{ return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void Hexapod_dev_InitializeDataMapInfo ( void ) { rtwCAPI_SetVersion ( ( *
rt_dataMapInfoPtr ) . mmi , 1 ) ; rtwCAPI_SetStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , & mmiStatic ) ; rtwCAPI_SetLoggingStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ; rtwCAPI_SetDataAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtDataAddrMap ) ; rtwCAPI_SetVarDimsAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtVarDimsAddrMap ) ;
rtwCAPI_SetInstanceLoggingInfo ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArray ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( ( * rt_dataMapInfoPtr ) . mmi , 0 ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void Hexapod_dev_host_InitializeDataMapInfo ( Hexapod_dev_host_DataMapInfo_T
* dataMap , const char * path ) { rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ;
rtwCAPI_SetStaticMap ( dataMap -> mmi , & mmiStatic ) ;
rtwCAPI_SetDataAddressMap ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , ( NULL ) ) ; rtwCAPI_SetPath
( dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , ( NULL ) )
; rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
