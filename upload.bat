@echo off

set FICHIER=src\main.ino  
set CARTE=arduino:avr:uno  
set PORT=COM3             

echo Compilation de %FICHIER%...
arduino-cli compile --fqbn %CARTE% %FICHIER%

if %errorlevel% neq 0 (
  echo Erreur de compilation
  exit /b 1
)

echo Téléversement de %FICHIER% sur le port %PORT%...
arduino-cli upload -p %PORT% --fqbn %CARTE% %FICHIER%

if %errorlevel% neq 0 (
  echo Erreur de téléversement
  exit /b 1
)

echo Téléversement réussi !
