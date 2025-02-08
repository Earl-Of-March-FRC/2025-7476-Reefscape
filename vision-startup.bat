@REM Environment variables
@set "USER=pi"
@set "ADDRESS=pi@10.178.28.1"
@set "PASSWORD=raspberry"
@set "REMOTE_DIR=/home/%USER%/2025_7476_reefscape"
@set "PY_DIRECTORY=limelight"

@REM @REM Pull from GitHub (subject to change)
@REM @echo Attempt to pull from GitHub (if possible)
@REM @git pull

@REM Store SSH credentials
@echo Storing SSH credentials
@cmdkey /generic:%ADDRESS% /user:%USER% /pass:%PASSWORD%

@REM Create directory on the Pi
@echo Creating directory on the Pi
@ssh %ADDRESS% "rm -rf %REMOTE_DIR% && mkdir -p %REMOTE_DIR%" || goto end

@REM Clone the Python module onto the Pi
@echo Cloning Python module through SSH
@scp -r %PY_DIRECTORY% %ADDRESS%:%REMOTE_DIR% || goto end

@REM @REM Install requirements and run the module using -m
@REM @echo Installing PIP requirements and running the module
@REM @ssh %ADDRESS% "cd %REMOTE_DIR%/%PY_DIRECTORY% && pip install -r requirements.txt && python3 -m python.algae.algaeClassification" || goto end

@REM Setup the Pi to run the script on startup using cron
@echo Setting up the script to run on startup
@ssh %ADDRESS% "echo '@reboot python3 -m python.algae.algaeClassification' | crontab -" || goto end

:end
pause
exit 0