@echo off

mkdir out
cd out
jar xf ../FtcDashboard-debug.aar
cd ..
echo Extracted aar

Rem                 ↓ Replace with new dash web socket url
java ReplaceUrl "c0d1135109c1.ngrok.io"
echo Replaced url

jar cvf FtcDashboard-debug2.aar -C out/ .
rmdir out /Q /S
echo Zipped aar

cd ..
move "%cd%\dash-aar\FtcDashboard-debug2.aar" "%cd%\libs"
echo Done
pause