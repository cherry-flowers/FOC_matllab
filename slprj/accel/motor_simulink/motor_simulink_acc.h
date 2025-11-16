#ifndef RTW_HEADER_motor_simulink_acc_h_
#define RTW_HEADER_motor_simulink_acc_h_
#include <stddef.h>
#include <float.h>
#ifndef motor_simulink_acc_COMMON_INCLUDES_
#define motor_simulink_acc_COMMON_INCLUDES_
#include <stdlib.h>
#define S_FUNCTION_NAME simulink_only_sfcn
#define S_FUNCTION_LEVEL 2
#ifndef RTW_GENERATED_S_FUNCTION
#define RTW_GENERATED_S_FUNCTION
#endif
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#endif
#include "motor_simulink_acc_types.h"
#include "multiword_types.h"
#include "mwmathutil.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
typedef struct { real_T B_11_0_0 ; real_T B_11_2_0 ; real_T B_11_3_0 ; real_T
B_11_4_0 ; real_T B_11_5_0 ; real_T B_11_6_0 [ 2 ] ; real_T B_11_6_1 [ 12 ] ;
real_T B_11_7_0 ; real_T B_11_14_0 ; real_T B_11_15_0 ; real_T B_11_18_0 [ 2
] ; real_T B_11_20_0 ; real_T B_11_22_0 ; real_T B_11_27_0 ; real_T B_11_30_0
; real_T B_11_33_0 ; real_T B_11_35_0 ; real_T B_11_36_0 ; real_T B_11_38_0 ;
real_T B_11_39_0 ; real_T B_11_50_0 [ 6 ] ; real_T B_11_57_0 ; real_T
B_11_69_0 ; real_T B_11_71_0 ; real_T B_11_73_0 ; real_T B_11_74_0 ; real_T
B_11_75_0 ; real_T B_11_77_0 ; real_T B_11_83_0 ; real_T B_11_84_0 ; real_T
B_11_87_0 ; real_T B_11_89_0 ; real_T B_11_91_0 ; real_T B_11_92_0 ; real_T
B_11_93_0 ; real_T B_11_100_0 ; real_T B_11_101_0 ; real_T B_11_102_0 ;
real_T B_11_105_0 ; real_T B_11_114_0 ; real_T B_11_115_0 ; real_T B_11_116_0
; real_T B_11_117_0 ; real_T B_11_118_0 ; real_T B_11_119_0 ; real_T
B_11_120_0 ; real_T B_11_121_0 ; real_T B_11_122_0 ; real_T B_11_123_0 ;
real_T B_11_124_0 ; real_T B_11_125_0 ; real_T B_11_126_0 ; real_T B_11_128_0
; real_T B_11_129_0 ; real_T B_11_130_0 ; real_T B_11_131_0 ; real_T
B_11_0_0_m ; real_T B_11_1_0 ; real_T B_11_3_0_c ; real_T B_11_4_0_k ; real_T
B_11_5_0_c ; real_T B_11_6_0_b ; real_T B_11_7_0_p ; real_T B_11_8_0 ; real_T
B_11_9_0 ; real_T B_11_10_0 ; real_T B_11_11_0 ; real_T B_11_13_0 ; real_T
B_11_15_0_c ; real_T B_11_16_0 ; real_T B_11_19_0 ; real_T B_11_25_0 ; real_T
B_11_26_0 ; real_T B_10_0_1 ; real_T B_10_0_2 ; real_T B_9_0_1 ; real_T
B_9_0_2 ; real_T B_8_0_0 ; real_T B_8_1_0 ; real_T B_7_0_0 ; real_T B_7_1_0 ;
real_T B_6_0_1 ; real_T B_5_0_1 ; real_T B_5_0_2 ; real_T B_5_0_3 ; real_T
B_5_0_4 ; real_T B_5_0_5 ; real_T B_5_0_6 ; real_T B_4_0_1 ; real_T B_4_0_2 ;
real_T B_4_0_3 ; real_T B_4_0_4 ; real_T B_4_0_5 ; real_T B_3_0_1 ; real_T
B_3_0_2 ; real_T B_3_0_3 ; real_T B_3_0_4 ; real_T B_3_0_5 ; real_T B_2_0_1 ;
real_T B_2_0_2 ; real_T B_1_0_1 ; real_T B_1_0_2 ; real_T B_0_0_1 ; real_T
B_0_0_2 ; real_T B_11_11_0_f [ 3 ] ; uint8_T B_11_22_0_g ; uint8_T B_11_24_0
; boolean_T B_11_42_0 ; boolean_T B_11_43_0 ; boolean_T B_11_45_0 ; boolean_T
B_11_46_0 ; boolean_T B_11_48_0 ; boolean_T B_11_49_0 ; } B_motor_simulink_T
; typedef struct { real_T Integrator_DSTATE ; real_T Filter_DSTATE ; real_T
Integrator_DSTATE_e ; real_T Filter_DSTATE_k ; real_T Integrator_DSTATE_o ;
real_T Filter_DSTATE_f ; real_T StateSpace_RWORK [ 2 ] ; struct { real_T
modelTStart ; } TransportDelay_RWORK ; struct { real_T modelTStart ; }
TransportDelay1_RWORK ; void * StateSpace_PWORK [ 22 ] ; struct { void *
TUbufferPtrs [ 2 ] ; } TransportDelay_PWORK ; struct { void * TUbufferPtrs [
2 ] ; } TransportDelay1_PWORK ; void * Lq_Ld_PWORK [ 2 ] ; void *
Scope1_PWORK ; void * Te_PWORK ; void * Wr_PWORK ; void * clark_idq_PWORK [ 2
] ; void * el_angle_PWORK ; void * hifi_in_PWORK [ 2 ] ; void * lowpass_PWORK
[ 2 ] ; void * lowpassRs_PWORK [ 2 ] ; void * pid_d_PWORK ; void *
pid_q_PWORK ; void * pwm_PWORK ; void * real_idq_PWORK ; void *
speed_monitor_PWORK [ 2 ] ; void * theta_PWORK ; void * theta_control_PWORK [
2 ] ; void * Scope_PWORK [ 3 ] ; void * Scope2_PWORK ; void * Scope3_PWORK ;
void * Scope1_PWORK_j ; void * Scope2_PWORK_d [ 2 ] ; void * iabc_PWORK ;
int32_T clockTickCounter ; int32_T counter ; int32_T counter_n ; int32_T
park_rs_sysIdxToRun ; int32_T park_sysIdxToRun ; int32_T
Subsystem1_sysIdxToRun ; int32_T Subsystempi2delay_sysIdxToRun ; int32_T
MATLABFunction_sysIdxToRun ; int32_T mid_point_svpwm_sysIdxToRun ; int32_T
MATLABFunction1_sysIdxToRun ; int32_T MATLABFunction_sysIdxToRun_e ; int32_T
Batterworth_rs_sysIdxToRun ; int32_T Batterworth_sysIdxToRun ; int32_T
MATLABFunction_sysIdxToRun_p ; int_T StateSpace_IWORK [ 23 ] ; struct { int_T
Tail ; int_T Head ; int_T Last ; int_T CircularBufSize ; int_T MaxNewBufSize
; } TransportDelay_IWORK ; struct { int_T Tail ; int_T Head ; int_T Last ;
int_T CircularBufSize ; int_T MaxNewBufSize ; } TransportDelay1_IWORK ; char
StateSpace_MODE [ 28 ] ; int_T Saturation_MODE ; int_T Saturation_MODE_g ;
int_T Sign_MODE ; int8_T Subsystem1_SubsysRanBC ; int8_T
Subsystempi2delay_SubsysRanBC ; boolean_T RelationalOperator3_Mode ;
boolean_T RelationalOperator_Mode ; boolean_T RelationalOperator2_Mode ;
boolean_T RelationalOperator1_Mode ; boolean_T RelationalOperator5_Mode ;
boolean_T RelationalOperator4_Mode ; boolean_T RelationalOperator1_Mode_d ;
boolean_T RelationalOperator2_Mode_n ; boolean_T RelationalOperator3_Mode_j ;
boolean_T RelationalOperator4_Mode_m ; boolean_T RelationalOperator5_Mode_h ;
boolean_T RelationalOperator6_Mode ; boolean_T Subsystem1_MODE ; boolean_T
Subsystempi2delay_MODE ; char_T pad_Subsystempi2delay_MODE [ 4 ] ; }
DW_motor_simulink_T ; typedef struct { real_T Int1_CSTATE ; real_T iq_CSTATE
; real_T id_CSTATE ; real_T Int_CSTATE ; } X_motor_simulink_T ; typedef
struct { real_T Int1_CSTATE ; real_T iq_CSTATE ; real_T id_CSTATE ; real_T
Int_CSTATE ; } XDot_motor_simulink_T ; typedef struct { boolean_T Int1_CSTATE
; boolean_T iq_CSTATE ; boolean_T id_CSTATE ; boolean_T Int_CSTATE ; }
XDis_motor_simulink_T ; typedef struct { real_T Int1_CSTATE ; real_T
iq_CSTATE ; real_T id_CSTATE ; real_T Int_CSTATE ; }
CStateAbsTol_motor_simulink_T ; typedef struct { real_T Int1_CSTATE ; real_T
iq_CSTATE ; real_T id_CSTATE ; real_T Int_CSTATE ; } CXPtMin_motor_simulink_T
; typedef struct { real_T Int1_CSTATE ; real_T iq_CSTATE ; real_T id_CSTATE ;
real_T Int_CSTATE ; } CXPtMax_motor_simulink_T ; typedef struct { real_T
StateSpace_Sf0_ZC [ 7 ] ; real_T RelationalOperator3_RelopInput_ZC ; real_T
RelationalOperator_RelopInput_ZC ; real_T RelationalOperator2_RelopInput_ZC ;
real_T RelationalOperator1_RelopInput_ZC ; real_T
RelationalOperator5_RelopInput_ZC ; real_T RelationalOperator4_RelopInput_ZC
; real_T Saturation_UprLim_ZC ; real_T Saturation_LwrLim_ZC ; real_T
Saturation_UprLim_ZC_c ; real_T Saturation_LwrLim_ZC_e ; real_T
RelationalOperator1_RelopInput_ZC_h ; real_T
RelationalOperator2_RelopInput_ZC_n ; real_T
RelationalOperator3_RelopInput_ZC_o ; real_T
RelationalOperator4_RelopInput_ZC_j ; real_T
RelationalOperator5_RelopInput_ZC_j ; real_T
RelationalOperator6_RelopInput_ZC ; real_T Sign_Input_ZC ; }
ZCV_motor_simulink_T ; typedef struct { ZCSigState StateSpace_Sf0_ZCE [ 7 ] ;
ZCSigState RelationalOperator3_RelopInput_ZCE ; ZCSigState
RelationalOperator_RelopInput_ZCE ; ZCSigState
RelationalOperator2_RelopInput_ZCE ; ZCSigState
RelationalOperator1_RelopInput_ZCE ; ZCSigState
RelationalOperator5_RelopInput_ZCE ; ZCSigState
RelationalOperator4_RelopInput_ZCE ; ZCSigState Saturation_UprLim_ZCE ;
ZCSigState Saturation_LwrLim_ZCE ; ZCSigState Saturation_UprLim_ZCE_m ;
ZCSigState Saturation_LwrLim_ZCE_j ; ZCSigState
RelationalOperator1_RelopInput_ZCE_k ; ZCSigState
RelationalOperator2_RelopInput_ZCE_l ; ZCSigState
RelationalOperator3_RelopInput_ZCE_o ; ZCSigState
RelationalOperator4_RelopInput_ZCE_g ; ZCSigState
RelationalOperator5_RelopInput_ZCE_i ; ZCSigState
RelationalOperator6_RelopInput_ZCE ; ZCSigState Sign_Input_ZCE ; }
PrevZCX_motor_simulink_T ; struct P_motor_simulink_T_ { real_T P_0 [ 2 ] ;
real_T P_1 [ 2 ] ; real_T P_2 ; real_T P_3 ; real_T P_4 ; real_T P_5 [ 2 ] ;
real_T P_6 [ 136 ] ; real_T P_7 [ 2 ] ; real_T P_8 [ 4 ] ; real_T P_9 [ 2 ] ;
real_T P_11 [ 2 ] ; real_T P_12 [ 748 ] ; real_T P_13 [ 2 ] ; real_T P_14 [
12 ] ; real_T P_15 [ 2 ] ; real_T P_16 [ 6 ] ; real_T P_17 [ 2 ] ; real_T
P_18 [ 6 ] ; real_T P_19 [ 2 ] ; real_T P_20 [ 6 ] ; real_T P_21 [ 2 ] ;
real_T P_22 ; real_T P_23 [ 2 ] ; real_T P_24 ; real_T P_25 ; real_T P_26 ;
real_T P_27 ; real_T P_28 ; real_T P_29 [ 9 ] ; real_T P_30 ; real_T P_31 ;
real_T P_32 ; real_T P_33 ; real_T P_34 ; real_T P_35 [ 3 ] ; real_T P_36 [ 3
] ; real_T P_37 ; real_T P_38 ; real_T P_39 ; real_T P_40 ; real_T P_41 ;
real_T P_42 ; real_T P_43 ; real_T P_44 ; real_T P_45 ; real_T P_46 ; real_T
P_47 ; real_T P_48 ; real_T P_49 ; real_T P_50 ; real_T P_51 ; real_T P_52 ;
real_T P_53 ; real_T P_54 ; real_T P_55 ; real_T P_56 ; real_T P_57 ; real_T
P_58 ; real_T P_59 ; real_T P_60 ; real_T P_61 ; real_T P_62 ; real_T P_63 ;
real_T P_64 ; real_T P_65 ; real_T P_66 ; real_T P_67 ; real_T P_68 ; real_T
P_69 ; real_T P_70 ; real_T P_71 ; real_T P_72 ; real_T P_73 ; real_T P_74 ;
real_T P_75 ; real_T P_76 ; real_T P_77 ; real_T P_78 ; real_T P_79 ; real_T
P_80 ; real_T P_81 ; real_T P_82 ; real_T P_83 ; real_T P_84 ; real_T P_85 ;
real_T P_86 ; real_T P_87 ; real_T P_88 ; real_T P_89 ; real_T P_90 ; real_T
P_91 ; real_T P_92 ; real_T P_93 ; real_T P_94 ; real_T P_95 ; real_T P_96 ;
real_T P_97 ; real_T P_98 ; real_T P_99 ; real_T P_100 ; real_T P_101 ;
real_T P_102 ; real_T P_103 ; real_T P_104 ; real_T P_105 ; } ; extern
P_motor_simulink_T motor_simulink_rtDefaultP ;
#endif
