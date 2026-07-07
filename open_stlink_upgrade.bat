@echo off
set "JAVA=%~dp0.tools\java17\jdk-17.0.19+10-jre\bin\javaw.exe"
set "UPGRADER=C:\Users\29525\.eide\tools\st_cube_programer\Drivers\FirmwareUpgrade\STLinkUpgrade.jar"

if not exist "%JAVA%" (
  echo Java runtime not found:
  echo %JAVA%
  pause
  exit /b 1
)

if not exist "%UPGRADER%" (
  echo ST-LINK upgrade tool not found:
  echo %UPGRADER%
  pause
  exit /b 1
)

start "" "%JAVA%" -jar "%UPGRADER%"
