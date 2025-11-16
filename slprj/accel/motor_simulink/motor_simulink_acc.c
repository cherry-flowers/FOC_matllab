#include <math.h>
#include "motor_simulink_acc.h"
#include "motor_simulink_acc_private.h"
#include <stdio.h>
#include "slexec_vm_simstruct_bridge.h"
#include "slexec_vm_zc_functions.h"
#include "slexec_vm_lookup_functions.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "simtarget/slSimTgtMdlrefSfcnBridge.h"
#include "simstruc.h"
#include "fixedpoint.h"
#define CodeFormat S-Function
#define AccDefine1 Accelerator_S-Function
#include "simtarget/slAccSfcnBridge.h"
#ifndef __RTW_UTFREE__  
extern void * utMalloc ( size_t ) ; extern void utFree ( void * ) ;
#endif
boolean_T motor_simulink_acc_rt_TDelayUpdateTailOrGrowBuf ( int_T * bufSzPtr
, int_T * tailPtr , int_T * headPtr , int_T * lastPtr , real_T tMinusDelay ,
real_T * * uBufPtr , boolean_T isfixedbuf , boolean_T istransportdelay ,
int_T * maxNewBufSzPtr ) { int_T testIdx ; int_T tail = * tailPtr ; int_T
bufSz = * bufSzPtr ; real_T * tBuf = * uBufPtr + bufSz ; real_T * xBuf = (
NULL ) ; int_T numBuffer = 2 ; if ( istransportdelay ) { numBuffer = 3 ; xBuf
= * uBufPtr + 2 * bufSz ; } testIdx = ( tail < ( bufSz - 1 ) ) ? ( tail + 1 )
: 0 ; if ( ( tMinusDelay <= tBuf [ testIdx ] ) && ! isfixedbuf ) { int_T j ;
real_T * tempT ; real_T * tempU ; real_T * tempX = ( NULL ) ; real_T * uBuf =
* uBufPtr ; int_T newBufSz = bufSz + 1024 ; if ( newBufSz > * maxNewBufSzPtr
) { * maxNewBufSzPtr = newBufSz ; } tempU = ( real_T * ) utMalloc ( numBuffer
* newBufSz * sizeof ( real_T ) ) ; if ( tempU == ( NULL ) ) { return ( false
) ; } tempT = tempU + newBufSz ; if ( istransportdelay ) tempX = tempT +
newBufSz ; for ( j = tail ; j < bufSz ; j ++ ) { tempT [ j - tail ] = tBuf [
j ] ; tempU [ j - tail ] = uBuf [ j ] ; if ( istransportdelay ) tempX [ j -
tail ] = xBuf [ j ] ; } for ( j = 0 ; j < tail ; j ++ ) { tempT [ j + bufSz -
tail ] = tBuf [ j ] ; tempU [ j + bufSz - tail ] = uBuf [ j ] ; if (
istransportdelay ) tempX [ j + bufSz - tail ] = xBuf [ j ] ; } if ( * lastPtr
> tail ) { * lastPtr -= tail ; } else { * lastPtr += ( bufSz - tail ) ; } *
tailPtr = 0 ; * headPtr = bufSz ; utFree ( uBuf ) ; * bufSzPtr = newBufSz ; *
uBufPtr = tempU ; } else { * tailPtr = testIdx ; } return ( true ) ; } real_T
motor_simulink_acc_rt_TDelayInterpolate ( real_T tMinusDelay , real_T tStart
, real_T * uBuf , int_T bufSz , int_T * lastIdx , int_T oldestIdx , int_T
newIdx , real_T initOutput , boolean_T discrete , boolean_T
minorStepAndTAtLastMajorOutput ) { int_T i ; real_T yout , t1 , t2 , u1 , u2
; real_T * tBuf = uBuf + bufSz ; if ( ( newIdx == 0 ) && ( oldestIdx == 0 )
&& ( tMinusDelay > tStart ) ) return initOutput ; if ( tMinusDelay <= tStart
) return initOutput ; if ( ( tMinusDelay <= tBuf [ oldestIdx ] ) ) { if (
discrete ) { return ( uBuf [ oldestIdx ] ) ; } else { int_T tempIdx =
oldestIdx + 1 ; if ( oldestIdx == bufSz - 1 ) tempIdx = 0 ; t1 = tBuf [
oldestIdx ] ; t2 = tBuf [ tempIdx ] ; u1 = uBuf [ oldestIdx ] ; u2 = uBuf [
tempIdx ] ; if ( t2 == t1 ) { if ( tMinusDelay >= t2 ) { yout = u2 ; } else {
yout = u1 ; } } else { real_T f1 = ( t2 - tMinusDelay ) / ( t2 - t1 ) ;
real_T f2 = 1.0 - f1 ; yout = f1 * u1 + f2 * u2 ; } return yout ; } } if (
minorStepAndTAtLastMajorOutput ) { if ( newIdx != 0 ) { if ( * lastIdx ==
newIdx ) { ( * lastIdx ) -- ; } newIdx -- ; } else { if ( * lastIdx == newIdx
) { * lastIdx = bufSz - 1 ; } newIdx = bufSz - 1 ; } } i = * lastIdx ; if (
tBuf [ i ] < tMinusDelay ) { while ( tBuf [ i ] < tMinusDelay ) { if ( i ==
newIdx ) break ; i = ( i < ( bufSz - 1 ) ) ? ( i + 1 ) : 0 ; } } else { while
( tBuf [ i ] >= tMinusDelay ) { i = ( i > 0 ) ? i - 1 : ( bufSz - 1 ) ; } i =
( i < ( bufSz - 1 ) ) ? ( i + 1 ) : 0 ; } * lastIdx = i ; if ( discrete ) {
double tempEps = ( DBL_EPSILON ) * 128.0 ; double localEps = tempEps *
muDoubleScalarAbs ( tBuf [ i ] ) ; if ( tempEps > localEps ) { localEps =
tempEps ; } localEps = localEps / 2.0 ; if ( tMinusDelay >= ( tBuf [ i ] -
localEps ) ) { yout = uBuf [ i ] ; } else { if ( i == 0 ) { yout = uBuf [
bufSz - 1 ] ; } else { yout = uBuf [ i - 1 ] ; } } } else { if ( i == 0 ) {
t1 = tBuf [ bufSz - 1 ] ; u1 = uBuf [ bufSz - 1 ] ; } else { t1 = tBuf [ i -
1 ] ; u1 = uBuf [ i - 1 ] ; } t2 = tBuf [ i ] ; u2 = uBuf [ i ] ; if ( t2 ==
t1 ) { if ( tMinusDelay >= t2 ) { yout = u2 ; } else { yout = u1 ; } } else {
real_T f1 = ( t2 - tMinusDelay ) / ( t2 - t1 ) ; real_T f2 = 1.0 - f1 ; yout
= f1 * u1 + f2 * u2 ; } } return ( yout ) ; } real_T look1_binlxpw ( real_T
u0 , const real_T bp0 [ ] , const real_T table [ ] , uint32_T maxIndex ) {
real_T frac ; real_T yL_0d0 ; uint32_T bpIdx ; uint32_T iLeft ; uint32_T
iRght ; if ( u0 <= bp0 [ 0U ] ) { iLeft = 0U ; frac = ( u0 - bp0 [ 0U ] ) / (
bp0 [ 1U ] - bp0 [ 0U ] ) ; } else if ( u0 < bp0 [ maxIndex ] ) { bpIdx =
maxIndex >> 1U ; iLeft = 0U ; iRght = maxIndex ; while ( iRght - iLeft > 1U )
{ if ( u0 < bp0 [ bpIdx ] ) { iRght = bpIdx ; } else { iLeft = bpIdx ; }
bpIdx = ( iRght + iLeft ) >> 1U ; } frac = ( u0 - bp0 [ iLeft ] ) / ( bp0 [
iLeft + 1U ] - bp0 [ iLeft ] ) ; } else { iLeft = maxIndex - 1U ; frac = ( u0
- bp0 [ maxIndex - 1U ] ) / ( bp0 [ maxIndex ] - bp0 [ maxIndex - 1U ] ) ; }
yL_0d0 = table [ iLeft ] ; return ( table [ iLeft + 1U ] - yL_0d0 ) * frac +
yL_0d0 ; } void rt_ssGetBlockPath ( SimStruct * S , int_T sysIdx , int_T
blkIdx , char_T * * path ) { _ssGetBlockPath ( S , sysIdx , blkIdx , path ) ;
} void rt_ssSet_slErrMsg ( void * S , void * diag ) { SimStruct * castedS = (
SimStruct * ) S ; if ( ! _ssIsErrorStatusAslErrMsg ( castedS ) ) {
_ssSet_slErrMsg ( castedS , diag ) ; } else { _ssDiscardDiagnostic ( castedS
, diag ) ; } } void rt_ssReportDiagnosticAsWarning ( void * S , void * diag )
{ _ssReportDiagnosticAsWarning ( ( SimStruct * ) S , diag ) ; } void
rt_ssReportDiagnosticAsInfo ( void * S , void * diag ) {
_ssReportDiagnosticAsInfo ( ( SimStruct * ) S , diag ) ; } static void
mdlOutputs ( SimStruct * S , int_T tid ) { real_T B_11_19_0 ; real_T
B_11_21_0 ; B_motor_simulink_T * _rtB ; DW_motor_simulink_T * _rtDW ;
P_motor_simulink_T * _rtP ; X_motor_simulink_T * _rtX ; real_T rtb_B_11_13_0
[ 3 ] ; real_T rtb_B_11_1_0 ; real_T rtb_B_11_1_1 ; real_T rtb_B_11_67_0 ;
real_T rtb_B_11_68_0 ; int32_T isHit ; _rtDW = ( ( DW_motor_simulink_T * )
ssGetRootDWork ( S ) ) ; _rtX = ( ( X_motor_simulink_T * ) ssGetContStates (
S ) ) ; _rtP = ( ( P_motor_simulink_T * ) ssGetModelRtp ( S ) ) ; _rtB = ( (
B_motor_simulink_T * ) _ssGetModelBlockIO ( S ) ) ; _rtB -> B_11_0_0 = _rtX
-> Int1_CSTATE ; muDoubleScalarSinCos ( _rtB -> B_11_0_0 , & rtb_B_11_1_0 , &
rtb_B_11_1_1 ) ; _rtB -> B_11_2_0 = _rtX -> iq_CSTATE ; _rtB -> B_11_3_0 =
_rtX -> id_CSTATE ; _rtB -> B_11_4_0 = _rtB -> B_11_2_0 * rtb_B_11_1_1 + _rtB
-> B_11_3_0 * rtb_B_11_1_0 ; _rtB -> B_11_5_0 = ( ( - _rtB -> B_11_2_0 -
1.7320508075688772 * _rtB -> B_11_3_0 ) * rtb_B_11_1_1 + ( 1.7320508075688772
* _rtB -> B_11_2_0 - _rtB -> B_11_3_0 ) * rtb_B_11_1_0 ) * 0.5 ;
ssCallAccelRunBlock ( S , 11 , 6 , SS_CALL_MDL_OUTPUTS ) ; isHit =
ssIsSampleHit ( S , 4 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_11_7_0 = ( _rtDW
-> clockTickCounter < _rtP -> P_27 ) && ( _rtDW -> clockTickCounter >= 0 ) ?
_rtP -> P_25 : 0.0 ; if ( _rtDW -> clockTickCounter >= _rtP -> P_26 - 1.0 ) {
_rtDW -> clockTickCounter = 0 ; } else { _rtDW -> clockTickCounter ++ ; }
ssCallAccelRunBlock ( S , 6 , 0 , SS_CALL_MDL_OUTPUTS ) ; } isHit =
ssIsSampleHit ( S , 3 , 0 ) ; if ( isHit != 0 ) { ssCallAccelRunBlock ( S , 3
, 0 , SS_CALL_MDL_OUTPUTS ) ; } _rtB -> B_11_11_0_f [ 0 ] = _rtB -> B_11_4_0
; _rtB -> B_11_11_0_f [ 1 ] = _rtB -> B_11_5_0 ; _rtB -> B_11_11_0_f [ 2 ] =
( 0.0 - _rtB -> B_11_5_0 ) - _rtB -> B_11_4_0 ; for ( isHit = 0 ; isHit < 3 ;
isHit ++ ) { rtb_B_11_13_0 [ isHit ] = ( ( _rtP -> P_29 [ isHit + 3 ] * _rtB
-> B_11_11_0_f [ 1 ] + _rtP -> P_29 [ isHit ] * _rtB -> B_11_11_0_f [ 0 ] ) +
_rtP -> P_29 [ isHit + 6 ] * _rtB -> B_11_11_0_f [ 2 ] ) * _rtP -> P_30 ; }
_rtB -> B_11_14_0 = ( _rtB -> B_11_0_0 - 1.5707963267948966 ) / 7.0 ; _rtB ->
B_11_15_0 = _rtB -> B_11_14_0 * _rtB -> B_11_9_0 ; isHit = ssIsSampleHit ( S
, 1 , 0 ) ; if ( ( isHit != 0 ) && ( ssIsMajorTimeStep ( S ) != 0 ) ) { if (
_rtB -> B_11_22_0_g > 0 ) { if ( ! _rtDW -> Subsystem1_MODE ) { if (
ssGetTaskTime ( S , 1 ) != ssGetTStart ( S ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( S ) ; } _rtDW -> Subsystem1_MODE
= true ; } } else if ( _rtDW -> Subsystem1_MODE ) {
ssSetBlockStateForSolverChangedAtMajorStep ( S ) ; _rtDW -> Subsystem1_MODE =
false ; } } if ( _rtDW -> Subsystem1_MODE ) { _rtB -> B_8_0_0 = rtb_B_11_13_0
[ 0 ] * muDoubleScalarCos ( _rtB -> B_11_15_0 ) + rtb_B_11_13_0 [ 1 ] *
muDoubleScalarSin ( _rtB -> B_11_15_0 ) ; _rtB -> B_8_1_0 = - rtb_B_11_13_0 [
0 ] * muDoubleScalarSin ( _rtB -> B_11_15_0 ) + rtb_B_11_13_0 [ 1 ] *
muDoubleScalarCos ( _rtB -> B_11_15_0 ) ; if ( ssIsMajorTimeStep ( S ) != 0 )
{ srUpdateBC ( _rtDW -> Subsystem1_SubsysRanBC ) ; } } isHit = ssIsSampleHit
( S , 1 , 0 ) ; if ( ( isHit != 0 ) && ( ssIsMajorTimeStep ( S ) != 0 ) ) {
if ( _rtB -> B_11_24_0 > 0 ) { if ( ! _rtDW -> Subsystempi2delay_MODE ) { if
( ssGetTaskTime ( S , 1 ) != ssGetTStart ( S ) ) {
ssSetBlockStateForSolverChangedAtMajorStep ( S ) ; } _rtDW ->
Subsystempi2delay_MODE = true ; } } else if ( _rtDW -> Subsystempi2delay_MODE
) { ssSetBlockStateForSolverChangedAtMajorStep ( S ) ; _rtDW ->
Subsystempi2delay_MODE = false ; } } if ( _rtDW -> Subsystempi2delay_MODE ) {
_rtB -> B_7_0_0 = rtb_B_11_13_0 [ 0 ] * muDoubleScalarSin ( _rtB -> B_11_15_0
) - rtb_B_11_13_0 [ 1 ] * muDoubleScalarCos ( _rtB -> B_11_15_0 ) ; _rtB ->
B_7_1_0 = rtb_B_11_13_0 [ 0 ] * muDoubleScalarCos ( _rtB -> B_11_15_0 ) +
rtb_B_11_13_0 [ 1 ] * muDoubleScalarSin ( _rtB -> B_11_15_0 ) ; if (
ssIsMajorTimeStep ( S ) != 0 ) { srUpdateBC ( _rtDW ->
Subsystempi2delay_SubsysRanBC ) ; } } if ( _rtB -> B_11_22_0_g != 0 ) { _rtB
-> B_11_18_0 [ 0 ] = _rtB -> B_8_0_0 ; _rtB -> B_11_18_0 [ 1 ] = _rtB ->
B_8_1_0 ; } else { _rtB -> B_11_18_0 [ 0 ] = _rtB -> B_7_0_0 ; _rtB ->
B_11_18_0 [ 1 ] = _rtB -> B_7_1_0 ; } { real_T * * uBuffer = ( real_T * * ) &
_rtDW -> TransportDelay_PWORK . TUbufferPtrs [ 0 ] ; real_T simTime = ssGetT
( S ) ; real_T tMinusDelay = simTime - _rtP -> P_31 ; B_11_19_0 =
motor_simulink_acc_rt_TDelayInterpolate ( tMinusDelay , 0.0 , * uBuffer ,
_rtDW -> TransportDelay_IWORK . CircularBufSize , & _rtDW ->
TransportDelay_IWORK . Last , _rtDW -> TransportDelay_IWORK . Tail , _rtDW ->
TransportDelay_IWORK . Head , _rtP -> P_32 , 1 , ( boolean_T ) (
ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) )
) ; } _rtB -> B_11_20_0 = _rtB -> B_11_18_0 [ 0 ] * B_11_19_0 ; { real_T * *
uBuffer = ( real_T * * ) & _rtDW -> TransportDelay1_PWORK . TUbufferPtrs [ 0
] ; real_T simTime = ssGetT ( S ) ; real_T tMinusDelay = simTime - _rtP ->
P_33 ; B_11_21_0 = motor_simulink_acc_rt_TDelayInterpolate ( tMinusDelay ,
0.0 , * uBuffer , _rtDW -> TransportDelay1_IWORK . CircularBufSize , & _rtDW
-> TransportDelay1_IWORK . Last , _rtDW -> TransportDelay1_IWORK . Tail ,
_rtDW -> TransportDelay1_IWORK . Head , _rtP -> P_34 , 1 , ( boolean_T ) (
ssIsMinorTimeStep ( S ) && ( ssGetTimeOfLastOutput ( S ) == ssGetT ( S ) ) )
) ; } _rtB -> B_11_22_0 = _rtB -> B_11_18_0 [ 1 ] * B_11_21_0 ; isHit =
ssIsSampleHit ( S , 3 , 0 ) ; if ( isHit != 0 ) { ssCallAccelRunBlock ( S , 1
, 0 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 0 , 0 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 11 , 25 ,
SS_CALL_MDL_OUTPUTS ) ; } isHit = ssIsSampleHit ( S , 4 , 0 ) ; if ( isHit !=
0 ) { ssCallAccelRunBlock ( S , 11 , 26 , SS_CALL_MDL_OUTPUTS ) ; } _rtB ->
B_11_27_0 = ( 0.0 * _rtB -> B_11_2_0 * _rtB -> B_11_3_0 + 0.00185 * _rtB ->
B_11_2_0 ) * 10.5 ; ssCallAccelRunBlock ( S , 11 , 28 , SS_CALL_MDL_OUTPUTS )
; _rtB -> B_11_33_0 = look1_binlxpw ( muDoubleScalarRem ( ssGetT ( S ) - _rtB
-> B_11_30_0 , _rtB -> B_11_15_0_c ) , _rtP -> P_36 , _rtP -> P_35 , 2U ) ;
isHit = ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_11_35_0 =
( muDoubleScalarSin ( ( ( real_T ) _rtDW -> counter + _rtP -> P_40 ) * 2.0 *
3.1415926535897931 / _rtP -> P_39 ) * _rtP -> P_37 + _rtP -> P_38 ) * _rtB ->
B_6_0_1 ; _rtB -> B_11_36_0 = _rtB -> B_11_35_0 ; _rtB -> B_11_38_0 = (
muDoubleScalarSin ( ( ( real_T ) _rtDW -> counter_n + _rtP -> P_44 ) * 2.0 *
3.1415926535897931 / _rtP -> P_43 ) * _rtP -> P_41 + _rtP -> P_42 ) * _rtB ->
B_6_0_1 ; _rtB -> B_11_39_0 = _rtB -> B_11_38_0 ; ssCallAccelRunBlock ( S , 5
, 0 , SS_CALL_MDL_OUTPUTS ) ; } isHit = ssIsSampleHit ( S , 1 , 0 ) ; if (
isHit != 0 ) { if ( ssIsMajorTimeStep ( S ) != 0 ) { _rtDW ->
RelationalOperator3_Mode = ( _rtB -> B_11_33_0 >= _rtB -> B_5_0_4 ) ; } _rtB
-> B_11_42_0 = ! _rtDW -> RelationalOperator3_Mode ; if ( ssIsMajorTimeStep (
S ) != 0 ) { _rtDW -> RelationalOperator_Mode = ( _rtB -> B_11_33_0 >= _rtB
-> B_5_0_1 ) ; } _rtB -> B_11_43_0 = _rtDW -> RelationalOperator_Mode ; if (
ssIsMajorTimeStep ( S ) != 0 ) { _rtDW -> RelationalOperator2_Mode = ( _rtB
-> B_11_33_0 >= _rtB -> B_5_0_5 ) ; } _rtB -> B_11_45_0 = ! _rtDW ->
RelationalOperator2_Mode ; if ( ssIsMajorTimeStep ( S ) != 0 ) { _rtDW ->
RelationalOperator1_Mode = ( _rtB -> B_11_33_0 >= _rtB -> B_5_0_2 ) ; } _rtB
-> B_11_46_0 = _rtDW -> RelationalOperator1_Mode ; if ( ssIsMajorTimeStep ( S
) != 0 ) { _rtDW -> RelationalOperator5_Mode = ( _rtB -> B_11_33_0 >= _rtB ->
B_5_0_6 ) ; } _rtB -> B_11_48_0 = ! _rtDW -> RelationalOperator5_Mode ; if (
ssIsMajorTimeStep ( S ) != 0 ) { _rtDW -> RelationalOperator4_Mode = ( _rtB
-> B_11_33_0 >= _rtB -> B_5_0_3 ) ; } _rtB -> B_11_49_0 = _rtDW ->
RelationalOperator4_Mode ; _rtB -> B_11_50_0 [ 0 ] = _rtB -> B_11_42_0 ; _rtB
-> B_11_50_0 [ 1 ] = _rtB -> B_11_43_0 ; _rtB -> B_11_50_0 [ 2 ] = _rtB ->
B_11_45_0 ; _rtB -> B_11_50_0 [ 3 ] = _rtB -> B_11_46_0 ; _rtB -> B_11_50_0 [
4 ] = _rtB -> B_11_48_0 ; _rtB -> B_11_50_0 [ 5 ] = _rtB -> B_11_49_0 ; }
_rtB -> B_11_57_0 = _rtX -> Int_CSTATE ; ssCallAccelRunBlock ( S , 11 , 58 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 11 , 59 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 11 , 60 ,
SS_CALL_MDL_OUTPUTS ) ; isHit = ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0
) { ssCallAccelRunBlock ( S , 11 , 61 , SS_CALL_MDL_OUTPUTS ) ; } isHit =
ssIsSampleHit ( S , 3 , 0 ) ; if ( isHit != 0 ) { ssCallAccelRunBlock ( S ,
11 , 62 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 4 , 0 ,
SS_CALL_MDL_OUTPUTS ) ; } ssCallAccelRunBlock ( S , 10 , 0 ,
SS_CALL_MDL_OUTPUTS ) ; isHit = ssIsSampleHit ( S , 3 , 0 ) ; if ( isHit != 0
) { ssCallAccelRunBlock ( S , 2 , 0 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 11 , 66 , SS_CALL_MDL_OUTPUTS ) ; } rtb_B_11_67_0 =
_rtB -> B_11_19_0 - _rtB -> B_11_18_0 [ 0 ] ; isHit = ssIsSampleHit ( S , 2 ,
0 ) ; if ( isHit != 0 ) { _rtB -> B_11_69_0 = _rtDW -> Integrator_DSTATE ; }
isHit = ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_11_71_0 =
_rtDW -> Filter_DSTATE ; } _rtB -> B_11_73_0 = ( _rtP -> P_49 * rtb_B_11_67_0
- _rtB -> B_11_71_0 ) * _rtP -> P_52 ; _rtB -> B_11_74_0 = ( _rtP -> P_46 *
rtb_B_11_67_0 + _rtB -> B_11_69_0 ) + _rtB -> B_11_73_0 ; if (
ssIsMajorTimeStep ( S ) != 0 ) { _rtDW -> Saturation_MODE = _rtB -> B_11_74_0
>= _rtP -> P_53 ? 1 : _rtB -> B_11_74_0 > _rtP -> P_54 ? 0 : - 1 ; } _rtB ->
B_11_75_0 = _rtDW -> Saturation_MODE == 1 ? _rtP -> P_53 : _rtDW ->
Saturation_MODE == - 1 ? _rtP -> P_54 : _rtB -> B_11_74_0 ;
ssCallAccelRunBlock ( S , 11 , 76 , SS_CALL_MDL_OUTPUTS ) ; _rtB -> B_11_77_0
= _rtB -> B_11_25_0 - _rtB -> B_11_57_0 ; isHit = ssIsSampleHit ( S , 2 , 0 )
; if ( isHit != 0 ) { _rtB -> B_11_83_0 = ( _rtP -> P_58 * _rtB -> B_11_77_0
- _rtDW -> Filter_DSTATE_k ) * _rtP -> P_61 ; _rtB -> B_11_84_0 = ( _rtP ->
P_55 * _rtB -> B_11_77_0 + _rtDW -> Integrator_DSTATE_e ) + _rtB -> B_11_83_0
; } rtb_B_11_68_0 = _rtB -> B_11_84_0 - _rtB -> B_11_18_0 [ 1 ] ; isHit =
ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) { _rtB -> B_11_87_0 = _rtDW
-> Integrator_DSTATE_o ; } isHit = ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit
!= 0 ) { _rtB -> B_11_89_0 = _rtDW -> Filter_DSTATE_f ; } _rtB -> B_11_91_0 =
( _rtP -> P_65 * rtb_B_11_68_0 - _rtB -> B_11_89_0 ) * _rtP -> P_68 ; _rtB ->
B_11_92_0 = ( _rtP -> P_62 * rtb_B_11_68_0 + _rtB -> B_11_87_0 ) + _rtB ->
B_11_91_0 ; if ( ssIsMajorTimeStep ( S ) != 0 ) { _rtDW -> Saturation_MODE_g
= _rtB -> B_11_92_0 >= _rtP -> P_69 ? 1 : _rtB -> B_11_92_0 > _rtP -> P_70 ?
0 : - 1 ; } _rtB -> B_11_93_0 = _rtDW -> Saturation_MODE_g == 1 ? _rtP ->
P_69 : _rtDW -> Saturation_MODE_g == - 1 ? _rtP -> P_70 : _rtB -> B_11_92_0 ;
ssCallAccelRunBlock ( S , 11 , 94 , SS_CALL_MDL_OUTPUTS ) ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { ssCallAccelRunBlock ( S ,
11 , 95 , SS_CALL_MDL_OUTPUTS ) ; } ssCallAccelRunBlock ( S , 11 , 96 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 11 , 97 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 11 , 98 ,
SS_CALL_MDL_OUTPUTS ) ; isHit = ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0
) { ssCallAccelRunBlock ( S , 11 , 99 , SS_CALL_MDL_OUTPUTS ) ; } _rtB ->
B_11_100_0 = _rtP -> P_71 * rtb_B_11_67_0 ; _rtB -> B_11_101_0 = _rtP -> P_72
* rtb_B_11_68_0 ; isHit = ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) {
_rtB -> B_11_102_0 = _rtP -> P_73 * _rtB -> B_11_77_0 ; }
muDoubleScalarSinCos ( _rtB -> B_11_0_0 , & rtb_B_11_67_0 , & rtb_B_11_68_0 )
; _rtB -> B_11_105_0 = _rtP -> P_74 * muDoubleScalarAtan2 ( rtb_B_11_67_0 ,
rtb_B_11_68_0 ) ; isHit = ssIsSampleHit ( S , 1 , 0 ) ; if ( ( isHit != 0 )
&& ( ssIsMajorTimeStep ( S ) != 0 ) ) { _rtDW -> RelationalOperator1_Mode_d =
( _rtB -> B_11_105_0 <= _rtB -> B_11_4_0_k ) ; _rtDW ->
RelationalOperator2_Mode_n = ( _rtB -> B_11_105_0 >= _rtB -> B_11_3_0_c ) ;
_rtDW -> RelationalOperator3_Mode_j = ( _rtB -> B_11_105_0 <= _rtB ->
B_11_6_0_b ) ; _rtDW -> RelationalOperator4_Mode_m = ( _rtB -> B_11_105_0 >=
_rtB -> B_11_5_0_c ) ; _rtDW -> RelationalOperator5_Mode_h = ( _rtB ->
B_11_105_0 <= _rtB -> B_11_8_0 ) ; _rtDW -> RelationalOperator6_Mode = ( _rtB
-> B_11_105_0 >= _rtB -> B_11_7_0_p ) ; } _rtB -> B_11_114_0 = ( ( 2.0 * _rtB
-> B_11_6_0 [ 0 ] + _rtB -> B_11_6_0 [ 1 ] ) * rtb_B_11_1_0 + -
1.7320508075688772 * _rtB -> B_11_6_0 [ 1 ] * rtb_B_11_1_1 ) *
0.33333333333333331 * _rtP -> P_75 ; _rtB -> B_11_115_0 = _rtP -> P_76 * _rtB
-> B_11_57_0 ; _rtB -> B_11_116_0 = _rtB -> B_11_115_0 * _rtB -> B_11_2_0 ;
_rtB -> B_11_117_0 = _rtP -> P_77 * _rtB -> B_11_116_0 ; _rtB -> B_11_118_0 =
_rtP -> P_78 * _rtB -> B_11_3_0 ; _rtB -> B_11_119_0 = ( _rtB -> B_11_114_0 -
_rtB -> B_11_118_0 ) + _rtB -> B_11_117_0 ; _rtB -> B_11_120_0 = ( ( 2.0 *
_rtB -> B_11_6_0 [ 0 ] + _rtB -> B_11_6_0 [ 1 ] ) * rtb_B_11_1_1 +
1.7320508075688772 * _rtB -> B_11_6_0 [ 1 ] * rtb_B_11_1_0 ) *
0.33333333333333331 * _rtP -> P_79 ; _rtB -> B_11_121_0 = _rtB -> B_11_3_0 *
_rtB -> B_11_115_0 ; _rtB -> B_11_122_0 = _rtP -> P_80 * _rtB -> B_11_121_0 ;
_rtB -> B_11_123_0 = _rtP -> P_81 * _rtB -> B_11_2_0 ; _rtB -> B_11_124_0 =
_rtP -> P_82 * _rtB -> B_11_115_0 ; _rtB -> B_11_125_0 = ( ( _rtB ->
B_11_120_0 - _rtB -> B_11_123_0 ) - _rtB -> B_11_122_0 ) - _rtB -> B_11_124_0
; _rtB -> B_11_126_0 = _rtP -> P_83 * _rtB -> B_11_57_0 ; isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { if ( _rtB -> B_11_57_0 >
0.0 ) { _rtDW -> Sign_MODE = 1 ; } else if ( _rtB -> B_11_57_0 < 0.0 ) {
_rtDW -> Sign_MODE = - 1 ; } else { _rtDW -> Sign_MODE = 0 ; } _rtB ->
B_11_128_0 = _rtP -> P_84 * ( real_T ) _rtDW -> Sign_MODE ; } _rtB ->
B_11_129_0 = _rtB -> B_11_128_0 + _rtB -> B_11_126_0 ; _rtB -> B_11_130_0 =
_rtB -> B_11_27_0 - _rtB -> B_11_129_0 ; _rtB -> B_11_131_0 = _rtP -> P_85 *
_rtB -> B_11_130_0 ; isHit = ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 )
{ ssCallAccelRunBlock ( S , 11 , 137 , SS_CALL_MDL_OUTPUTS ) ; } isHit =
ssIsSampleHit ( S , 1 , 0 ) ; if ( isHit != 0 ) { ssCallAccelRunBlock ( S ,
11 , 138 , SS_CALL_MDL_OUTPUTS ) ; } ssCallAccelRunBlock ( S , 11 , 139 ,
SS_CALL_MDL_OUTPUTS ) ; isHit = ssIsSampleHit ( S , 4 , 0 ) ; if ( isHit != 0
) { ssCallAccelRunBlock ( S , 11 , 140 , SS_CALL_MDL_OUTPUTS ) ; } isHit =
ssIsSampleHit ( S , 2 , 0 ) ; if ( isHit != 0 ) { ssCallAccelRunBlock ( S ,
11 , 141 , SS_CALL_MDL_OUTPUTS ) ; } ssCallAccelRunBlock ( S , 9 , 0 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 11 , 143 ,
SS_CALL_MDL_OUTPUTS ) ; UNUSED_PARAMETER ( tid ) ; } static void
mdlOutputsTID5 ( SimStruct * S , int_T tid ) { B_motor_simulink_T * _rtB ;
P_motor_simulink_T * _rtP ; _rtP = ( ( P_motor_simulink_T * ) ssGetModelRtp (
S ) ) ; _rtB = ( ( B_motor_simulink_T * ) _ssGetModelBlockIO ( S ) ) ; _rtB
-> B_11_0_0_m = _rtP -> P_86 ; _rtB -> B_11_1_0 = _rtP -> P_87 ; _rtB ->
B_11_3_0_c = _rtP -> P_88 ; _rtB -> B_11_4_0_k = _rtP -> P_89 ; _rtB ->
B_11_5_0_c = _rtP -> P_90 ; _rtB -> B_11_6_0_b = _rtP -> P_91 ; _rtB ->
B_11_7_0_p = _rtP -> P_92 ; _rtB -> B_11_8_0 = _rtP -> P_93 ; _rtB ->
B_11_9_0 = _rtP -> P_94 ; _rtB -> B_11_10_0 = _rtP -> P_95 ; _rtB ->
B_11_11_0 = _rtP -> P_96 ; _rtB -> B_11_13_0 = _rtP -> P_97 ; _rtB ->
B_11_15_0_c = _rtP -> P_98 ; _rtB -> B_11_16_0 = _rtP -> P_99 ; _rtB ->
B_11_19_0 = _rtP -> P_100 ; _rtB -> B_11_22_0_g = ( uint8_T ) ( _rtP -> P_101
== _rtP -> P_102 ) ; _rtB -> B_11_24_0 = ( uint8_T ) ( _rtP -> P_101 == _rtP
-> P_103 ) ; _rtB -> B_11_25_0 = _rtP -> P_104 ; _rtB -> B_11_26_0 = _rtP ->
P_105 ; UNUSED_PARAMETER ( tid ) ; }
#define MDL_UPDATE
static void mdlUpdate ( SimStruct * S , int_T tid ) { B_motor_simulink_T *
_rtB ; DW_motor_simulink_T * _rtDW ; P_motor_simulink_T * _rtP ; int32_T
isHit ; _rtDW = ( ( DW_motor_simulink_T * ) ssGetRootDWork ( S ) ) ; _rtP = (
( P_motor_simulink_T * ) ssGetModelRtp ( S ) ) ; _rtB = ( (
B_motor_simulink_T * ) _ssGetModelBlockIO ( S ) ) ; ssCallAccelRunBlock ( S ,
11 , 6 , SS_CALL_MDL_UPDATE ) ; { real_T * * uBuffer = ( real_T * * ) & _rtDW
-> TransportDelay_PWORK . TUbufferPtrs [ 0 ] ; real_T simTime = ssGetT ( S )
; _rtDW -> TransportDelay_IWORK . Head = ( ( _rtDW -> TransportDelay_IWORK .
Head < ( _rtDW -> TransportDelay_IWORK . CircularBufSize - 1 ) ) ? ( _rtDW ->
TransportDelay_IWORK . Head + 1 ) : 0 ) ; if ( _rtDW -> TransportDelay_IWORK
. Head == _rtDW -> TransportDelay_IWORK . Tail ) { if ( !
motor_simulink_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay_IWORK . CircularBufSize , & _rtDW -> TransportDelay_IWORK .
Tail , & _rtDW -> TransportDelay_IWORK . Head , & _rtDW ->
TransportDelay_IWORK . Last , simTime - _rtP -> P_31 , uBuffer , ( boolean_T
) 0 , false , & _rtDW -> TransportDelay_IWORK . MaxNewBufSize ) ) {
ssSetErrorStatus ( S , "tdelay memory allocation error" ) ; return ; } } ( *
uBuffer + _rtDW -> TransportDelay_IWORK . CircularBufSize ) [ _rtDW ->
TransportDelay_IWORK . Head ] = simTime ; ( * uBuffer ) [ _rtDW ->
TransportDelay_IWORK . Head ] = _rtB -> B_11_35_0 ; } { real_T * * uBuffer =
( real_T * * ) & _rtDW -> TransportDelay1_PWORK . TUbufferPtrs [ 0 ] ; real_T
simTime = ssGetT ( S ) ; _rtDW -> TransportDelay1_IWORK . Head = ( ( _rtDW ->
TransportDelay1_IWORK . Head < ( _rtDW -> TransportDelay1_IWORK .
CircularBufSize - 1 ) ) ? ( _rtDW -> TransportDelay1_IWORK . Head + 1 ) : 0 )
; if ( _rtDW -> TransportDelay1_IWORK . Head == _rtDW ->
TransportDelay1_IWORK . Tail ) { if ( !
motor_simulink_acc_rt_TDelayUpdateTailOrGrowBuf ( & _rtDW ->
TransportDelay1_IWORK . CircularBufSize , & _rtDW -> TransportDelay1_IWORK .
Tail , & _rtDW -> TransportDelay1_IWORK . Head , & _rtDW ->
TransportDelay1_IWORK . Last , simTime - _rtP -> P_33 , uBuffer , ( boolean_T
) 0 , false , & _rtDW -> TransportDelay1_IWORK . MaxNewBufSize ) ) {
ssSetErrorStatus ( S , "tdelay memory allocation error" ) ; return ; } } ( *
uBuffer + _rtDW -> TransportDelay1_IWORK . CircularBufSize ) [ _rtDW ->
TransportDelay1_IWORK . Head ] = simTime ; ( * uBuffer ) [ _rtDW ->
TransportDelay1_IWORK . Head ] = _rtB -> B_11_35_0 ; } isHit = ssIsSampleHit
( S , 2 , 0 ) ; if ( isHit != 0 ) { _rtDW -> counter ++ ; if ( _rtDW ->
counter == _rtP -> P_39 ) { _rtDW -> counter = 0 ; } _rtDW -> counter_n ++ ;
if ( _rtDW -> counter_n == _rtP -> P_43 ) { _rtDW -> counter_n = 0 ; } _rtDW
-> Integrator_DSTATE += _rtP -> P_47 * _rtB -> B_11_100_0 ; _rtDW ->
Filter_DSTATE += _rtP -> P_50 * _rtB -> B_11_73_0 ; _rtDW ->
Integrator_DSTATE_e += _rtP -> P_56 * _rtB -> B_11_102_0 ; _rtDW ->
Filter_DSTATE_k += _rtP -> P_59 * _rtB -> B_11_83_0 ; _rtDW ->
Integrator_DSTATE_o += _rtP -> P_63 * _rtB -> B_11_101_0 ; _rtDW ->
Filter_DSTATE_f += _rtP -> P_66 * _rtB -> B_11_91_0 ; } UNUSED_PARAMETER (
tid ) ; }
#define MDL_UPDATE
static void mdlUpdateTID5 ( SimStruct * S , int_T tid ) { UNUSED_PARAMETER (
tid ) ; }
#define MDL_DERIVATIVES
static void mdlDerivatives ( SimStruct * S ) { B_motor_simulink_T * _rtB ;
DW_motor_simulink_T * _rtDW ; P_motor_simulink_T * _rtP ;
XDot_motor_simulink_T * _rtXdot ; _rtDW = ( ( DW_motor_simulink_T * )
ssGetRootDWork ( S ) ) ; _rtXdot = ( ( XDot_motor_simulink_T * ) ssGetdX ( S
) ) ; _rtP = ( ( P_motor_simulink_T * ) ssGetModelRtp ( S ) ) ; _rtB = ( (
B_motor_simulink_T * ) _ssGetModelBlockIO ( S ) ) ; _rtXdot -> Int1_CSTATE =
_rtB -> B_11_115_0 ; _rtXdot -> iq_CSTATE = _rtB -> B_11_125_0 ; _rtXdot ->
id_CSTATE = _rtB -> B_11_119_0 ; _rtXdot -> Int_CSTATE = _rtB -> B_11_131_0 ;
}
#define MDL_ZERO_CROSSINGS
static void mdlZeroCrossings ( SimStruct * S ) { B_motor_simulink_T * _rtB ;
DW_motor_simulink_T * _rtDW ; P_motor_simulink_T * _rtP ;
ZCV_motor_simulink_T * _rtZCSV ; _rtDW = ( ( DW_motor_simulink_T * )
ssGetRootDWork ( S ) ) ; _rtZCSV = ( ( ZCV_motor_simulink_T * )
ssGetSolverZcSignalVector ( S ) ) ; _rtP = ( ( P_motor_simulink_T * )
ssGetModelRtp ( S ) ) ; _rtB = ( ( B_motor_simulink_T * ) _ssGetModelBlockIO
( S ) ) ; ssCallAccelRunBlock ( S , 11 , 6 , SS_CALL_MDL_ZERO_CROSSINGS ) ;
_rtZCSV -> RelationalOperator3_RelopInput_ZC = _rtB -> B_11_33_0 - _rtB ->
B_5_0_4 ; _rtZCSV -> RelationalOperator_RelopInput_ZC = _rtB -> B_11_33_0 -
_rtB -> B_5_0_1 ; _rtZCSV -> RelationalOperator2_RelopInput_ZC = _rtB ->
B_11_33_0 - _rtB -> B_5_0_5 ; _rtZCSV -> RelationalOperator1_RelopInput_ZC =
_rtB -> B_11_33_0 - _rtB -> B_5_0_2 ; _rtZCSV ->
RelationalOperator5_RelopInput_ZC = _rtB -> B_11_33_0 - _rtB -> B_5_0_6 ;
_rtZCSV -> RelationalOperator4_RelopInput_ZC = _rtB -> B_11_33_0 - _rtB ->
B_5_0_3 ; _rtZCSV -> Saturation_UprLim_ZC = _rtB -> B_11_74_0 - _rtP -> P_53
; _rtZCSV -> Saturation_LwrLim_ZC = _rtB -> B_11_74_0 - _rtP -> P_54 ;
_rtZCSV -> Saturation_UprLim_ZC_c = _rtB -> B_11_92_0 - _rtP -> P_69 ;
_rtZCSV -> Saturation_LwrLim_ZC_e = _rtB -> B_11_92_0 - _rtP -> P_70 ;
_rtZCSV -> RelationalOperator1_RelopInput_ZC_h = _rtB -> B_11_105_0 - _rtB ->
B_11_4_0_k ; _rtZCSV -> RelationalOperator2_RelopInput_ZC_n = _rtB ->
B_11_105_0 - _rtB -> B_11_3_0_c ; _rtZCSV ->
RelationalOperator3_RelopInput_ZC_o = _rtB -> B_11_105_0 - _rtB -> B_11_6_0_b
; _rtZCSV -> RelationalOperator4_RelopInput_ZC_j = _rtB -> B_11_105_0 - _rtB
-> B_11_5_0_c ; _rtZCSV -> RelationalOperator5_RelopInput_ZC_j = _rtB ->
B_11_105_0 - _rtB -> B_11_8_0 ; _rtZCSV -> RelationalOperator6_RelopInput_ZC
= _rtB -> B_11_105_0 - _rtB -> B_11_7_0_p ; _rtZCSV -> Sign_Input_ZC = _rtB
-> B_11_57_0 ; } static void mdlInitializeSizes ( SimStruct * S ) {
ssSetChecksumVal ( S , 0 , 602133317U ) ; ssSetChecksumVal ( S , 1 ,
39930234U ) ; ssSetChecksumVal ( S , 2 , 2775959844U ) ; ssSetChecksumVal ( S
, 3 , 1379307132U ) ; { mxArray * slVerStructMat = ( NULL ) ; mxArray *
slStrMat = mxCreateString ( "simulink" ) ; char slVerChar [ 10 ] ; int status
= mexCallMATLAB ( 1 , & slVerStructMat , 1 , & slStrMat , "ver" ) ; if (
status == 0 ) { mxArray * slVerMat = mxGetField ( slVerStructMat , 0 ,
"Version" ) ; if ( slVerMat == ( NULL ) ) { status = 1 ; } else { status =
mxGetString ( slVerMat , slVerChar , 10 ) ; } } mxDestroyArray ( slStrMat ) ;
mxDestroyArray ( slVerStructMat ) ; if ( ( status == 1 ) || ( strcmp (
slVerChar , "10.4" ) != 0 ) ) { return ; } } ssSetOptions ( S ,
SS_OPTION_EXCEPTION_FREE_CODE ) ; if ( ssGetSizeofDWork ( S ) != sizeof (
DW_motor_simulink_T ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal DWork sizes do "
"not match for accelerator mex file." ) ; } if ( ssGetSizeofGlobalBlockIO ( S
) != sizeof ( B_motor_simulink_T ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal BlockIO sizes do "
"not match for accelerator mex file." ) ; } { int ssSizeofParams ;
ssGetSizeofParams ( S , & ssSizeofParams ) ; if ( ssSizeofParams != sizeof (
P_motor_simulink_T ) ) { static char msg [ 256 ] ; sprintf ( msg ,
"Unexpected error: Internal Parameters sizes do "
"not match for accelerator mex file." ) ; } } _ssSetModelRtp ( S , ( real_T *
) & motor_simulink_rtDefaultP ) ; rt_InitInfAndNaN ( sizeof ( real_T ) ) ; }
static void mdlInitializeSampleTimes ( SimStruct * S ) { { SimStruct * childS
; SysOutputFcn * callSysFcns ; childS = ssGetSFunction ( S , 0 ) ;
callSysFcns = ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ]
= ( SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 1 ) ; callSysFcns
= ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 2 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 3 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 4 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 5 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 6 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 7 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 8 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; } slAccRegPrmChangeFcn ( S , mdlOutputsTID5 ) ; }
static void mdlTerminate ( SimStruct * S ) { }
#include "simulink.c"
