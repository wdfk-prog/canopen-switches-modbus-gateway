@echo off
rem 声明采用UTF-8编码
@chcp 65001
::@echo off执行以后，后面所有的命令均不显示，包括本条命令。
@cd /d %~dp0
@start F:\BaiduNetdiskDownload\env-windows-v1.3.2\env-windows-v1.3.2\tools\ConEmu\ConEmu64.exe
::修改为env运行程序目录