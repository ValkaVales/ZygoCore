@echo off

set SOURCE=D:\VS\ZygoCore
set DEST0=D:\VS\ZygoGL

rem set DEST=D:\VS\TestZygoCore
rem set DEST=D:\VS\TitanBrusok
rem set DEST=D:\VS\TestPID
rem set DEST=D:\VS\PhysRigidBody
set DEST=D:\VS\_COPY4\PhysRigidBody
rem set DEST=D:\VS\_NeuroBook\NeuroBook00_COPY



rem set DEST=D:\VS\FlyingModels
rem set DEST=D:\VS\PhysTest
rem set DEST=D:\VS\_NeuroBook\CudaTest00
rem set DEST=D:\VS\_NeuroBook\NeuroBook00
rem set DEST=D:\VS\_NeuroBook\NeuroBook01_withCuda
rem set DEST=D:\VS\TestUartConnection
rem set DEST=D:\VS\_NeuralNetStudy_UsingPyTorch\Test000
rem set DEST=D:\VS\TurretWithAi
rem set DEST=D:\VS\RobotModels
rem set DEST=D:\VS\PhysSimpleTest2
rem set DEST=D:\VS\RandomNumberGenerator
rem set DEST=D:\VS\_ForPavel\04_NoAI_3D_pathFind
rem set DEST=D:\VS\_ForPavel\05_NoAI_3D_pathFind_without_OneRobot_SubData
rem set DEST=D:\VS\_ForPavel\06_NoAI_3D_pathFind_simultaniously_again
rem set DEST=D:\VS\_ForPavel\08_NoAI_3D_pathFind_all_times_simultaniously_fixed_graph
rem set DEST=D:\VS\_ForPavel\09_Graphs3DViewer
rem set DEST=D:\VS\_ForPavel\10_NoAI_3D_pathFind_all_times_simultaniously_only_graph
rem set DEST=D:\VS\_ForPavel\11_Graphs3DViewer_for10
rem set DEST=D:\VS\_ForPavel\12_Graph_Clasters
rem set DEST=D:\VS\_ForPavel\13_Graphs3DViewer_for12
rem set DEST=D:\VS\_CryptoMining\Bitcoin
rem set DEST=D:\VS\_NeuroBook\NeuroSha256
rem set DEST=D:\VS\Chess
rem set DEST=D:\VS\NiceVectorGraphicsSamples
rem set DEST=D:\VS\BitcoinSoloMiner


call:myMkDir %DEST%\lib
call:myMkDir %DEST%\lib_release

copy %SOURCE%\x64\Debug\zygocore.lib %DEST%\lib\
copy %SOURCE%\x64\Release\zygocore.lib %DEST%\lib_release\

copy %SOURCE%\x64\Debug\zygocore.lib %DEST0%\lib\
copy %SOURCE%\x64\Release\zygocore.lib %DEST0%\lib_release\

goto:eof

rem ------------------------------------------------------------------- functions

:myMkDir
IF NOT exist %~1 (
mkdir %~1
echo Dir created: %~1
)
goto:eof

