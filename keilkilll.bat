del *.bak /s
del *.ddk /s
del *.edk /s
del *.lst /s 
del *.lnp /s
del *.mpf /s
del *.mpj /s
del *.obj /s
del *.omf /s
::del *.opt /s  ::不允许删除JLINK的设置
del *.plg /s
del *.rpt /s
del *.tmp /s
del *.__i /s
del *.crf /s
del *.o /s
del *.d /s
del *.axf /s
del *.tra /s
del *.dep /s           
del JLinkLog.txt /s

del *.iex /s
::del *.htm /s
::del *.sct /s ::不允许删除链接的设置
del *.map /s
::删除cubemx生成无用工程文件
del C:\Users\Administrator\Desktop\STM32F4_RTT-rttv4.1.1\board\CubeMX_Config\MDK-ARM\CubeMX_Config.uvprojx
del C:\Users\Administrator\Desktop\STM32F4_RTT-rttv4.1.1\board\CubeMX_Config\MDK-ARM\CubeMX_Config.uvoptx
exit
