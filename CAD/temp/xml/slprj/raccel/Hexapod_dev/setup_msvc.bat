set "VSCMD_START_DIR=%CD%"
CALL "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\VCVARSALL.BAT " amd64
IF ERRORLEVEL 1 (EXIT /B 1)
