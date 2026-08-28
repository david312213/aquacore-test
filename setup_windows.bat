@echo off
setlocal

echo [1/4] Checking Python...
where py >nul 2>nul
if errorlevel 1 (
  echo Python launcher was not found.
  echo Install Python 3.11 or 3.12 from https://www.python.org/downloads/windows/
  echo During installation, enable "Add Python to PATH".
  exit /b 1
)

echo [2/4] Creating virtual environment...
py -3.11 -m venv .venv 2>nul
if errorlevel 1 py -3.12 -m venv .venv 2>nul
if errorlevel 1 py -3 -m venv .venv
if errorlevel 1 exit /b 1

echo [3/4] Installing dependencies...
call .venv\Scripts\activate.bat
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
if errorlevel 1 exit /b 1

echo [4/4] Running self-tests...
python -m unittest discover -s tests -v
if errorlevel 1 exit /b 1

echo.
echo Setup complete. Try:
echo   .venv\Scripts\python.exe run_sim.py --controller student --seed 1001 --animate
endlocal
