# Camera calibration #


### What is this repository for? ###

* This is the repository for calibration two individual camera
* 1.0


### Prepare Camera Calibration Chessboard ###

	Make chessboard like on white paper
	The black/white rectangle size is 25x25mm
	The grid is 9x7

### How to run? ###
- Test of camera
  Check the camera is running
  	```
  	set exe=bin2015\x64\release\stereo_calib.exe
	%exe% camera -w=1280 -h=480\
	```
- Make image for calibration
  	```
  	set exe=bin2015\x64\release\stereo_calib.exe
	%exe% image -d=calib -n=100
	```
	In above example, the 100 images are made and saved in calib directory.
	In this time the image which doesn’t contain Chessboard is ignored.
	The corrected image’s name are “image01.jpg”, “image02.jpg”and the size is 1280x480.
- Calculate calibration parameter
  	```
  	set exe=bin2015\x64\release\stereo_calib.exe
	%exe% calib -d=calib/ -nx=8 -nx=6 -s=2.5 –f=image-list.txt 
	or
	%exe% calib -d=calib/ -nx=8 -nx=6 -s=2.5
	```
	It calculate the calibration parameter with the images which are in image-list.txt.
	If don’t specify the image-list.txt, then all images are used in the directory.

	- -f : the file which contain the list of image file name
	- -s: the real size of black rectangle in Chessboard.
	- -nx, -ny: the interline count in Chessboard (if Chessboard is 9x7, then nx=8, ny=6)

	After calculation there will be camera_parametersXXX.xml file in the calib directory.
	The XXX is the count of referenced images.

	If the count of image is 100 then, it takes about 1~2 hours to calculate
- Test
  It show the calibration calculation result
  	```
  	set exe=bin2015\x64\release\stereo_calib.exe
	%exe% test -d=calib -p=calib/camera_params100.xml
	```

### How to apply calibration parameter file ###
	Rename the camera_parametersXXX.xml to the camera_parameters.xml
	Copy the camera_parameters.xml file to the board directory

	The path of board directory is ~/careye/models

