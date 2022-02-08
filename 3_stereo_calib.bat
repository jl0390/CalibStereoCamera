@echo off
set exe=bin2015\x64\release\stereo_calib.exe
set calibexe=bin2015\x64\release\stereo_calib_calib.exe
copy /Y %exe% %calibexe%
%calibexe% calib -d=calib/ -nx=8 -ny=6 -s=2.5
del %calibexe%
