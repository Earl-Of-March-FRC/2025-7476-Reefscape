@echo off

@REM Environment variables
@set "USER=pi"
@set "ADDRESS=%USER%@10.74.76.22"
@set "PASSWORD=raspberry"
@set "REMOTE_DIR=/home/%USER%/2025_7476_reefscape"
@set "PY_DIRECTORY=limelight"

@REM --------------------------------------------------------------------------------------------
@REM Create directory on the Pi
@echo [STEP 1] Creating directory on the Pi...
@echo.

:putty_create_dir
plink -ssh %ADDRESS% -pw %PASSWORD% "rm -rf %REMOTE_DIR% && mkdir -p %REMOTE_DIR%" || (
    call :putty_fail 
    goto ssh_create_dir
)
goto putty_clone_py

:ssh_create_dir
ssh %ADDRESS% "rm -rf %REMOTE_DIR% && mkdir -p %REMOTE_DIR%" || goto default_fail
goto putty_clone_py

@REM --------------------------------------------------------------------------------------------
@REM Clone the Python module onto the Pi
@echo [STEP 2] Cloning Python module through SSH...
@echo.

:putty_clone_py
pscp -pw %PASSWORD% -r %PY_DIRECTORY% %ADDRESS%:%REMOTE_DIR% || (
    call :putty_fail 
    goto scp_clone_py
)
goto putty_setup_startup

:scp_clone_py
scp -r %PY_DIRECTORY% %ADDRESS%:%REMOTE_DIR% || goto default_fail
goto putty_setup_startup

@REM --------------------------------------------------------------------------------------------
@REM Setup the Pi to run the script on startup using cron
@echo [STEP 3] Setting up the script to run on startup...
@echo.

:putty_setup_startup
plink -v -ssh %ADDRESS% -pw %PASSWORD% "cd %REMOTE_DIR%/%PY_DIRECTORY% && echo '@reboot python3 -m python.algae.orange_algae_classification' | crontab -" || (
    call :putty_fail 
    goto ssh_setup_startup
)
goto job_complete

:ssh_setup_startup
ssh %ADDRESS% "cd %REMOTE_DIR%/%PY_DIRECTORY% && echo '@reboot python3 -m python.algae.orange_algae_classification' | crontab -" || goto default_fail
goto job_complete

@REM --------------------------------------------------------------------------------------------
:job_complete
@echo.
@echo Batch job completed.
goto end

@REM --------------------------------------------------------------------------------------------
:end
@echo.
@echo.
@echo Press any key to close the batch.
@pause >nul
@exit 0

@REM --------------------------------------------------------------------------------------------
@REM Error message for PuTTY-related packages fail
:putty_fail
@echo.
@echo Failed to use PuTTY for the task. Will migrate to default powershell commands... Make sure you download the full PuTTY package from here: https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html
@echo.
@exit /b

@REM --------------------------------------------------------------------------------------------
@REM Error message when default PowerShell commands fail (and ending the batch)
:default_fail
@echo.
@echo Default commands have failed. Ending batch.
@echo.
goto end