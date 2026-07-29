@echo off
cd /d "%~dp0"
echo.
echo  Lunar Rover Web Sim
echo  Open: http://localhost:8080
echo  Press Ctrl+C to stop
echo.
start http://localhost:8080
python -m http.server 8080
