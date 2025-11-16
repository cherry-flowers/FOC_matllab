@echo off
set MATLAB=D:\MATLAB
"%MATLAB%\bin\win64\gmake" -f motor_simulink.mk  OPTS="-DTID01EQ=0"
