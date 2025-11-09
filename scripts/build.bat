
setlocal

@ECHO OFF

IF "%1"=="" (
  goto :usage
)

IF "%2"=="" (
  goto :usage
)

set "FUNCTION=%~1"
set "VARIANT=%~2"

echo FUNCTION: %FUNCTION%
echo VARIANT: %VARIANT%

set HEADER_FILE="vildehaye-common\config-%FUNCTION%-%VARIANT%.h"

if not exist "%HEADER_FILE%" (
    echo ERROR: File not founded: %HEADER_FILE%
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
    echo ERROR: Project not founded: %PROJECT%
    exit /b 1
)

cd "%PROJECT%"
dir

echo #ifndef CONFIG_H > config.h
echo #define CONFIG_H >> config.h

echo /* Automatically generated %DATE% %TIME% */ >> config.h

echo #include ^"../vildehaye-common/config-tag-v2-6.h^" >> config.h

echo #endif >> config.h

cd Debug 

"C:\\Programs\\ccs1010\\ccs\\utils\\bin\\gmake" -k -j 8 clean -O 
"C:\\Programs\\ccs1010\\ccs\\utils\\bin\\gmake" -k -j 8 all -O 

dir *.hex

goto :eof

:usage
ECHO scripts\build FUNCTION VARIANT
ECHO For example, scripts\build tag v2-6

