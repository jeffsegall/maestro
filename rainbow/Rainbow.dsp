# Microsoft Developer Studio Project File - Name="Rainbow" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Application" 0x0101

CFG=Rainbow - Win32 RTSS Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "Rainbow.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "Rainbow.mak" CFG="Rainbow - Win32 RTSS Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "Rainbow - Win32 RTSS Release" (based on "Win32 (x86) Application")
!MESSAGE "Rainbow - Win32 RTSS Debug" (based on "Win32 (x86) Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "Rainbow - Win32 RTSS Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Rainbow___Win32_RTSS_Release"
# PROP BASE Intermediate_Dir "Rainbow___Win32_RTSS_Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Rainbow___Win32_RTSS_Release"
# PROP Intermediate_Dir "Rainbow___Win32_RTSS_Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /ML /W3 /GX /Ox /I "$(RtxSdkDir)\include" /D "WIN32" /D "UNDER_RTSS" /D "NDEBUG" /D "RELEASE" /D "_UNICODE" /D "UNICODE" /YX /FD /c
# ADD CPP /nologo /ML /W3 /GX /Ox /I "$(RtxSdkDir)\include" /D "WIN32" /D "UNDER_RTSS" /D "NDEBUG" /D "RELEASE" /D "_UNICODE" /D "UNICODE" /YX /FD /c
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x412 /d "_DEBUG"
# ADD RSC /l 0x412 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 startupCRT.obj rtxlibc.lib oldnames.lib rtapi_rtss.lib rtx_rtss.lib /nologo /entry:"_RtapiProcessEntryCRT@8" /pdb:none /machine:I386 /nodefaultlib:"libc" /nodefaultlib:"libcd" /nodefaultlib:"libcmt" /nodefaultlib:"libcmtd" /nodefaultlib /out:"Rainbow___Win32_RTSS_Release/Rainbow.rtss" /libpath:"$(RtxSdkDir)\lib" /libpath:"$(RtxSdkDir)\lib\VisualStudio6" /driver /subsystem:native,4.00 /align:0x20
# ADD LINK32 startupCRT.obj rtxlibc.lib oldnames.lib rtapi_rtss.lib rtx_rtss.lib /nologo /entry:"_RtapiProcessEntryCRT@8" /pdb:none /machine:I386 /nodefaultlib:"libc" /nodefaultlib:"libcd" /nodefaultlib:"libcmt" /nodefaultlib:"libcmtd" /nodefaultlib /out:"Rainbow___Win32_RTSS_Release/Rainbow.rtss" /libpath:"$(RtxSdkDir)\lib" /libpath:"$(RtxSdkDir)\lib\VisualStudio6" /driver /subsystem:native,4.00 /align:0x20

!ELSEIF  "$(CFG)" == "Rainbow - Win32 RTSS Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Rainbow___Win32_RTSS_Debug"
# PROP BASE Intermediate_Dir "Rainbow___Win32_RTSS_Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Rainbow___Win32_RTSS_Debug"
# PROP Intermediate_Dir "Rainbow___Win32_RTSS_Debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /Z7 /Od /I "$(RtxSdkDir)\include" /D "WIN32" /D "_DEBUG" /D "UNDER_RTSS" /D "_UNICODE" /D "UNICODE" /YX /FD /c
# ADD CPP /nologo /W3 /GX /Z7 /Od /I "$(RtxSdkDir)\include" /D "WIN32" /D "_DEBUG" /D "UNDER_RTSS" /D "_UNICODE" /D "UNICODE" /YX /FD /c
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x412 /d "_DEBUG"
# ADD RSC /l 0x412 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 startupCRT.obj rtxlibc.lib oldnames.lib rtapi_rtss.lib rtx_rtss.lib /nologo /entry:"_RtapiProcessEntryCRT@8" /incremental:no /debug /debugtype:both /machine:I386 /nodefaultlib:"libc" /nodefaultlib:"libcd" /nodefaultlib:"libcmt" /nodefaultlib:"libcmtd" /nodefaultlib /out:"Rainbow___Win32_RTSS_Debug/Rainbow.rtss" /pdbtype:sept /libpath:"$(RtxSdkDir)\lib" /libpath:"$(RtxSdkDir)\lib\VisualStudio6" /driver /subsystem:native,4.00 /align:0x20
# SUBTRACT BASE LINK32 /pdb:none
# ADD LINK32 startupCRT.obj rtxlibc.lib oldnames.lib rtapi_rtss.lib rtx_rtss.lib /nologo /entry:"_RtapiProcessEntryCRT@8" /incremental:no /debug /debugtype:both /machine:I386 /nodefaultlib:"libc" /nodefaultlib:"libcd" /nodefaultlib:"libcmt" /nodefaultlib:"libcmtd" /nodefaultlib /out:"Rainbow___Win32_RTSS_Debug/Rainbow.rtss" /pdbtype:sept /libpath:"$(RtxSdkDir)\lib" /libpath:"$(RtxSdkDir)\lib\VisualStudio6" /driver /subsystem:native,4.00 /align:0x20
# SUBTRACT LINK32 /pdb:none

!ENDIF 

# Begin Target

# Name "Rainbow - Win32 RTSS Release"
# Name "Rainbow - Win32 RTSS Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=.\CAN.CPP
# End Source File
# Begin Source File

SOURCE=.\Core.cpp
# End Source File
# Begin Source File

SOURCE=.\mocap_functions.cpp
# End Source File
# Begin Source File

SOURCE=.\nrutil.cpp
# End Source File
# Begin Source File

SOURCE=.\RTX.CPP
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=.\CAN.H
# End Source File
# Begin Source File

SOURCE=.\CANID.H
# End Source File
# Begin Source File

SOURCE=.\CommonDefinition.h
# End Source File
# Begin Source File

SOURCE=.\mocap_functions.h
# End Source File
# Begin Source File

SOURCE=.\nrutil.h
# End Source File
# Begin Source File

SOURCE=.\RTX.H
# End Source File
# Begin Source File

SOURCE=.\SharedMemory.h
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# End Group
# End Target
# End Project
