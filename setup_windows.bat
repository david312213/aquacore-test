@echo off
setlocal
set "ENV_NAME=aquacore-test"

echo [1/3] Checking Conda...
where conda >nul 2>nul
if errorlevel 1 (
  echo Conda was not found.
  echo Install Miniconda, then run this script from Anaconda Prompt.
  echo Official installer: https://www.anaconda.com/docs/getting-started/miniconda/install/windows-gui-install
  exit /b 1
)

echo [2/3] Creating or updating %ENV_NAME%...
call conda run -n "%ENV_NAME%" python --version >nul 2>nul
if errorlevel 1 (
  call conda env create --file environment.yml
) else (
  call conda env update --name "%ENV_NAME%" --file environment.yml --prune
)
if errorlevel 1 exit /b 1

echo [3/3] Running self-tests...
call conda run -n "%ENV_NAME%" python -m unittest discover -s tests -v
if errorlevel 1 exit /b 1

echo.
echo Setup complete. Try:
echo   conda run -n %ENV_NAME% python run_sim.py --controller student --seed 1001 --animate --events
endlocal
