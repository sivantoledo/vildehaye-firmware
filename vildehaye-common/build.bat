echo off

for /D %%i in (..\vildehaye-*) do (
  echo copying %%i hex file
  copy %%i\Debug\*.hex ..\..\atlas\atlas-distribution\firmware
)