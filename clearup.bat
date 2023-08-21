@echo off

for /d %%a in (*) do (
  rd %~dp0%%a\build /S /Q
  rd %~dp0%%a\emStudio\Output /S /Q
)
echo DONE1
