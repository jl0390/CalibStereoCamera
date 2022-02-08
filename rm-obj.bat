del /Q /S *.aps
del /Q /S *.sdf
del /Q /S *.suo
del /Q /S *.pdb
del /Q /S *.ilk
del /Q /S *.exp

rmdir /Q /S bin\Int
rmdir /Q /S bin\x64\Debug
rmdir /Q /S bin\x86\Debug

rmdir /Q /S bin2015\Int
rmdir /Q /S bin2015\x64\Debug
rmdir /Q /S bin2015\x86\Debug

del /AH *.v12.suo
del /Q /S *.VC.db

pause
