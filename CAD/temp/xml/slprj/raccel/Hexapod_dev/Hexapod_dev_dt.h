#include "ext_types.h"
static DataTypeInfo rtDataTypeInfoTable [ ] = { { "real_T" , 0 , 8 } , {
"real32_T" , 1 , 4 } , { "int8_T" , 2 , 1 } , { "uint8_T" , 3 , 1 } , {
"int16_T" , 4 , 2 } , { "uint16_T" , 5 , 2 } , { "int32_T" , 6 , 4 } , {
"uint32_T" , 7 , 4 } , { "boolean_T" , 8 , 1 } , { "fcn_call_T" , 9 , 0 } , {
"int_T" , 10 , 4 } , { "pointer_T" , 11 , 8 } , { "action_T" , 12 , 8 } , {
"timer_uint32_pair_T" , 13 , 8 } , { "physical_connection" , 14 , 8 } , {
"int64_T" , 15 , 8 } , { "uint64_T" , 16 , 8 } , { "string" , 17 , 8 } , {
"string" , 18 , 8 } , { "struct_djHv1IDeD06XoprYU3B4T" , 19 , 64 } , {
"struct_PdEm5aibBpUO2JM4kXLMUF" , 20 , 120 } , {
"struct_j64StuZ2rTU6bLJIiNazJE" , 21 , 8 } , {
"struct_yi3YsCL79DNI0dtXLQLW7F" , 22 , 16 } , {
"struct_2zFvE4klqMEEpiiM95wYGC" , 23 , 32 } , {
"struct_NjJb0BLBoxXOwHw15Vx6hC" , 24 , 32 } , {
"struct_OdmtFjiCR5QAIiBTFdK6YF" , 25 , 40 } , {
"struct_gih9egjPD1GTXsGE36a93E" , 26 , 21872 } , { "uint64_T" , 27 , 8 } , {
"int64_T" , 28 , 8 } , { "uint_T" , 29 , 32 } , { "char_T" , 30 , 8 } , {
"uchar_T" , 31 , 8 } , { "time_T" , 32 , 8 } } ; static uint_T
rtDataTypeSizes [ ] = { sizeof ( real_T ) , sizeof ( real32_T ) , sizeof ( int8_T ) , sizeof ( uint8_T ) , sizeof ( int16_T ) , sizeof ( uint16_T ) , sizeof ( int32_T ) , sizeof ( uint32_T ) , sizeof ( boolean_T ) , sizeof ( fcn_call_T ) , sizeof ( int_T ) , sizeof ( pointer_T ) , sizeof ( action_T ) , 2 * sizeof ( uint32_T ) , sizeof ( int32_T ) , sizeof ( int64_T ) , sizeof ( uint64_T ) , 8 , sizeof ( char_T ) , sizeof ( struct_djHv1IDeD06XoprYU3B4T ) , sizeof ( struct_PdEm5aibBpUO2JM4kXLMUF ) , sizeof ( struct_j64StuZ2rTU6bLJIiNazJE ) , sizeof ( struct_yi3YsCL79DNI0dtXLQLW7F ) , sizeof ( struct_2zFvE4klqMEEpiiM95wYGC ) , sizeof ( struct_NjJb0BLBoxXOwHw15Vx6hC ) , sizeof ( struct_OdmtFjiCR5QAIiBTFdK6YF ) , sizeof ( struct_gih9egjPD1GTXsGE36a93E ) , sizeof ( uint64_T ) , sizeof ( int64_T ) , sizeof ( uint_T ) , sizeof ( char_T ) , sizeof ( uchar_T ) , sizeof ( time_T ) } ; static const char_T * rtDataTypeNames [ ] = { "real_T" , "real32_T" , "int8_T" , "uint8_T" , "int16_T" , "uint16_T" , "int32_T" , "uint32_T" , "boolean_T" , "fcn_call_T" , "int_T" , "pointer_T" , "action_T" , "timer_uint32_pair_T" , "physical_connection" , "int64_T" , "uint64_T" , "string" , "string" , "struct_djHv1IDeD06XoprYU3B4T" , "struct_PdEm5aibBpUO2JM4kXLMUF" , "struct_j64StuZ2rTU6bLJIiNazJE" , "struct_yi3YsCL79DNI0dtXLQLW7F" , "struct_2zFvE4klqMEEpiiM95wYGC" , "struct_NjJb0BLBoxXOwHw15Vx6hC" , "struct_OdmtFjiCR5QAIiBTFdK6YF" , "struct_gih9egjPD1GTXsGE36a93E" , "uint64_T" , "int64_T" , "uint_T" , "char_T" , "uchar_T" , "time_T" } ; static DataTypeTransition rtBTransitions [ ] = { { ( char_T * ) ( & rtB . aslpipoci2 ) , 0 , 0 , 98 } , { ( char_T * ) ( & rtDW . o35huj2gsd ) , 0 , 0 , 120054 } , { ( char_T * ) ( & rtDW . jfocsgvjr2 ) , 11 , 0 , 117 } , { ( char_T * ) ( & rtDW . fusdopd4md ) , 6 , 0 , 6 } , { ( char_T * ) ( & rtDW . bmm15pnlh2 ) , 7 , 0 , 6 } , { ( char_T * ) ( & rtDW . ee11ecv44v ) , 10 , 0 , 4 } , { ( char_T * ) ( & rtDW . irnshlasoh ) , 3 , 0 , 10 } , { ( char_T * ) ( & rtDW . bdszturt0n ) , 8 , 0 , 2 } } ; static DataTypeTransitionTable rtBTransTable = { 8U , rtBTransitions } ; static DataTypeTransition rtPTransitions [ ] = { { ( char_T * ) ( & rtP . time [ 0 ] ) , 0 , 0 , 70019 } , { ( char_T * ) ( & rtP . uDLookupTable_maxIndex ) , 7 , 0 , 24 } } ; static DataTypeTransitionTable rtPTransTable = { 2U , rtPTransitions } ;
