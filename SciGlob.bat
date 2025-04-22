@echo off
REM Set the path to your virtual environment's activate script
set VENV_PATH=C:\Users\Sciglob\Downloads\Polarizer_SW-main\.venv\Scripts\activate.bat

REM Set the path to your Python script
set SCRIPT_PATH=C:\Users\Sciglob\Downloads\Polarizer_SW-main\main.py

REM Activate the virtual environment
call "%VENV_PATH%"

REM Run the Python script
python "%SCRIPT_PATH%"

REM Deactivate the virtual environment (optional)
call "%VENV_PATH%" deactivate

REM Pause the command window to see the output (optional)
pause