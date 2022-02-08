@echo off
set exe=bin2015\x64\release\stereo_calib.exe
%exe% test -d=calib -p=calib/camera_params30.xml
