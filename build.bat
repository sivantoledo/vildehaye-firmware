
setlocal

@ECHO OFF

IF "%1"=="" (
  goto :usage
)

setlocal EnableDelayedExpansion
if "%1"=="list" (
    :: dir vildehaye-common\config-*.h
    for /f "delims=" %%A in ('dir "vildehaye-common\config-*.h" /b /a-d 2^>nul') do (
        set "filename=%%~nA"
        echo !filename:config-=!
    )
    goto :EOF
)

set "CONFIG=%~1"

echo CONFIG: %CONFIG%

set HEADER_FILE="vildehaye-common\config-%CONFIG%.h"

if not exist "%HEADER_FILE%" (
    echo ERROR: File not found: %HEADER_FILE%
    exit /b 1
)

for /f "usebackq tokens=3 delims= " %%A in (`findstr /i /c:"BUILD_CONFIG_ARCH" "%HEADER_FILE%" 2^>nul`) do (
    set "ARCH=%%A"
    goto :extracted_arch
)
echo ERROR: Could not find BUILD_CONFIG_ARCH in %HEADER_FILE%
exit /b 1

:extracted_arch
echo ARCHITECTURE: %ARCH%

for /f "usebackq tokens=3 delims= " %%A in (`findstr /i /c:"BUILD_CONFIG_SC" "%HEADER_FILE%" 2^>nul`) do (
    set "SC=%%A"
    goto :extracted_sc
)
echo ERROR: Could not find BUILD_CONFIG_SC in %HEADER_FILE%
exit /b 1

:extracted_sc
echo SENSOR CONTROLLER CODE: %SC%

set "PROJECT=vildehaye-%ARCH%-%SC%"
echo PROJECT: %PROJECT%

if not exist "%PROJECT%" (
    echo ERROR: Project not found: %PROJECT%
    exit /b 1
)

cd "%PROJECT%"
dir

:: echo #ifndef CONFIG_H > config.h
:: echo #define CONFIG_H >> config.h

echo /* Automatically generated %DATE% %TIME% */ > config.h

echo #include ^"../vildehaye-common/config-%CONFIG%.h^" >> config.h

:: secho #endif >> config.h

cd Debug 

"C:\\Programs\\ccs1010\\ccs\\utils\\bin\\gmake" -k -j 8 clean -O 
"C:\\Programs\\ccs1010\\ccs\\utils\\bin\\gmake" -k -j 8 all -O 

dir *.hex

ECHO copy %PROJECT%.hex ..\..\firmware\vildehaye-%CONFIG%.hex
copy %PROJECT%.hex ..\..\firmware\vildehaye-%CONFIG%.hex

goto :eof

:usage
ECHO build CONFIG
ECHO For example, build tag-v2-6
ECHO or,
ECHO build list

