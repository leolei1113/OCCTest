SET VCVER=vc14
SET OS_TYPE=win64

SET CASROOT=C:\OpenCASCADE-7.5.0-vc14-64\opencascade-7.5.0

SET D3D9=D:\Microsoft DirectX SDK (June 2010)
SET QTDIR=D:\Qt\5.12.0\msvc2017_64

SET PATH=%CASROOT%\%OS_TYPE%\%VCVER%\bin;%PATH%

start "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\devenv.exe" .\OCCTest.sln
