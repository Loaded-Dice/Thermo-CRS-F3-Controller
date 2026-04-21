@echo off
cd /d "%~dp0"
set YOSYS=C:\apio\packages\oss-cad-suite\bin\yosys.exe
if not exist build mkdir build
"%YOSYS%" -q -p "synth_ice40 -top rs485_Gateway -json build\rs485_Gateway.json" rs485_Gateway.v > build\syn_out.txt 2>&1
echo EXITCODE=%ERRORLEVEL% >> build\syn_out.txt
