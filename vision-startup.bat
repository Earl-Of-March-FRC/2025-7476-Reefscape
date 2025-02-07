@REM Environment variables
set "USER=pi"
set "ADDRESS=pi@10.178.28.1"
set "PASSWORD=raspberry"
set "REMOTE_DIR=/home/%USER%/2025_7476_reefscape"
set "PY_SCRIPT=limelight"

@REM Pull from GitHub (subject to change)
git pull

@REM Store credentials
cmdkey /generic:%ADDRESS% /user:%USER% /pass:%PASSWORD%

@REM Create directory on the pi
ssh %ADDRESS% "rm -rf %REMOTE_DIR% && mkdir -p %REMOTE_DIR%"

@REM Clone python module onto pi
scp -r %PY_SCRIPT% %ADDRESS%:%REMOTE_DIR%

ssh %ADDRESS% "cd %REMOTE_DIR%/%PY_SCRIPT% && pip install -r requirements.txt && python3 -m python.algae.algaeClassifcation"

pause