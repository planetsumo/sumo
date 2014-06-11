set SUMO_HOME=%CD%\..
call testEnv.bat %1
texttest.py -gx
