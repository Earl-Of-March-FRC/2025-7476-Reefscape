@REM Environment variables
@set "USER=pi"
@set "ADDRESS=pi@10.178.28.1"
@set "PASSWORD=raspberry"
@set "REMOTE_DIR=/home/%USER%/2025_7476_reefscape"
@set "PY_SCRIPT=limelight"

@REM @REM Pull from GitHub (subject to change)
@REM @echo Attempt to pull from GitHub (if possible)
@REM @git pull

@REM Store credentials
@echo Store SSH credentials
@cmdkey /generic:%ADDRESS% /user:%USER% /pass:%PASSWORD%

@REM Create directory on the pi
@echo Create directory onto the pi
@ssh %ADDRESS% "rm -rf %REMOTE_DIR% && mkdir -p %REMOTE_DIR%" || goto end

@REM Clone python module onto pi
@echo Clone python module through SSH
@scp -r %PY_SCRIPT% %ADDRESS%:%REMOTE_DIR% || goto end

@REM Run algaeClassification
@echo Run algaeClassification module
@ssh %ADDRESS% "cd %REMOTE_DIR%/%PY_SCRIPT% && python3 -m python.algae.algaeClassifcation" || goto end

:end

pause

exit 0

