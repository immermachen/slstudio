%%
% Build SLStudio Matlab MEX functions
% Jakob Wilm, DTU, 2014

srcDir = '../src/';

%% Camera
srcDirCamera = [srcDir 'camera/'];
if ismac
    CXXFLAGS = {'-I/opt/local/lib/'};
    LDFLAGS = 'LDFLAGS = "\$LDFLAGS -ldc1394 -lueye_api"';
    DEFINES = {'-DWITH_CAMERAIIDC'};
    srcFilesCamera = {'Camera.cpp', 'CameraIIDC.cpp'};
elseif isunix
    CXXFLAGS = {'-I/opt/Vimba_1_3/VimbaCPP/Include', '-I/opt/Vimba_1_3'};
    LDFLAGS = 'LDFLAGS = "\$LDFLAGS -L /opt/Vimba_1_3/VimbaCPP/DynamicLib/x86_64bit -lVimbaCPP -lVimbaC"';
    DEFINES = {'-DWITH_CAMERAVIMBA'};
    srcFilesCamera = {'Camera.cpp', 'CameraVimba.cpp'};
elseif ispc
% 	CXXFLAGS = {'-IC:/Program Files/IDS/uEye/Develop/include/'};
%   LDFLAGS = 'C:\Program Files\IDS\uEye\Develop\Lib\uEye_api_64.lib';
%   DEFINES = {'-DWITH_CAMERAIDSIMAGING'};
    CXXFLAGS = {'-IC:/Program Files/Point Grey Research/FlyCapture2/include/'};
    LDFLAGS = { '-lFlyCapture2', '-LC:/Program Files/Point Grey Research/FlyCapture2/lib64'};
    DEFINES = {'-DWITH_CAMERAPOINTGREY', '-DWIN64'};
    srcFilesCamera = {'Camera.cpp', 'CameraPointGrey.cpp'};
end

srcFilesCamera = strcat(srcDirCamera, srcFilesCamera);
srcFilesCamera = ['CameraMex.cpp' srcFilesCamera];
  
%mex('-v', ['-I' srcDirCamera], CXXFLAGS{:}, DEFINES{:}, srcFilesCamera{:}, LDFLAGS{:});
%mex('-v', ['-I' srcDirCamera], CXXFLAGS(:), DEFINES(:), srcFilesCamera(:), LDFLAGS(:));
%mex -v -I/home/yang/slstudio/src/camera/ -I/opt/Vimba_1_3/VimbaCPP/Include -I/opt/Vimba_1_3  -DWITH_CAMERAVIMBA  CameraMex.cpp    /home/yang/slstudio/src/camera/Bitmap.cpp /home/yang/slstudio/src/camera/Camera.cpp    /home/yang/slstudio/src/camera/CameraVimba.cpp  -L/opt/Vimba_1_3/VimbaCPP/DynamicLib/x86_64bit -lVimbaCPP -lVimbaC

%Ubuntu: Yang: only use this sentence code
mex -v -I/home/yang/slstudio/src/camera/ -I/opt/Vimba_1_3/VimbaCPP/Include -I/opt/Vimba_1_3  -DWITH_CAMERAVIMBA  CameraMex.cpp    /home/yang/slstudio/src/camera/Bitmap.cpp /home/yang/slstudio/src/camera/Camera.cpp    /home/yang/slstudio/src/camera/CameraVimba.cpp  /opt/Vimba_1_3/VimbaCPP/DynamicLib/x86_64bit/libVimbaCPP.so /opt/Vimba_1_3/VimbaCPP/DynamicLib/x86_64bit/libVimbaC.so

%Windows: Yang: only use this sentence code
%mex -v -ID:/slstudio/src/camera/ '-IC:/Users/Public/Documents/Allied Vision/Vimba_1.4/VimbaCPP_Examples/Common' '-IC:/Program Files/Allied Vision Technologies/AVTVimba_1.4/VimbaCPP/Include' '-IC:/Program Files/Allied Vision Technologies/AVTVimba_1.4'  -DWITH_CAMERAVIMBA  CameraMex.cpp    D:/slstudio/src/camera/Bitmap.cpp D:/slstudio/src/camera/Camera.cpp    D:/slstudio/src/camera/CameraVimba.cpp  '-LC:/Program Files/Allied Vision Technologies/AVTVimba_1.4/VimbaCPP/Lib/Win64/VimbaCPP.lib' '-LC:/Program Files/Allied Vision Technologies/AVTVimba_1.4/VimbaC/Lib/Win64/VimbaC.lib'

%% Projector
srcDirProjector = [srcDir 'projector/'];
srcFilesProjector = {'ProjectorOpenGL.cpp'};
if ismac
    srcFilesProjector = [srcFilesProjector 'OpenGLContext.Mac.cpp'];
    CXXFLAGS = {'CXXFLAGS = "\$CXXFLAGS -ObjC++"'};
    DEFINES = {};
    LDFLAGS = {'LDFLAGS = "\$LDFLAGS -framework Cocoa -framework OpenGL"'};
elseif isunix
    srcFilesProjector = [srcFilesProjector 'OpenGLContext.Unix.cpp'];
    CXXFLAGS = {'CXXFLAGS=$CXXFLAGS'};
    DEFINES = {};
    LDFLAGS = {'LINKLIBS=$LINKLIBS -lGL -lGLU -lX11 -lXxf86vm -lGLEW'};
elseif ispc
    srcFilesProjector = [srcFilesProjector 'OpenGLContext.Win.cpp'];
	CXXFLAGS = {'-IC:\Program Files\glew-1.13.0\include'};
    DEFINES = {'-DUNICODE'};
    LDFLAGS = {'-lOpenGL32', '-LC:\Program Files\glew-1.13.0\lib\Release\x64', '-lglew32'};
end

srcFilesProjector = strcat(srcDirProjector, srcFilesProjector);
srcFilesProjector = ['ProjectorMex.cpp' srcFilesProjector];

%Ubuntu: Yang: only use this sentence code
mex -v -I/home/yang/slstudio/src/projector/ ProjectorMex.cpp  /home/yang/slstudio/src/projector/OpenGLContext.Unix.cpp /home/yang/slstudio/src/projector/ProjectorOpenGL.cpp -lGL -lGLU -lX11 -lXxf86vm -lGLEW

%Windows: Yang: only use this sentence code
%mex -v -ID:/slstudio/src/camera/ -IC:/Program Files/Allied Vision Technologies/AVTVimba_1.4/VimbaCPP/Include -IC:/Program Files/Allied Vision Technologies/AVTVimba_1.4  -DWITH_CAMERAVIMBA  CameraMex.cpp    D:/slstudio/src/camera/Bitmap.cpp D:/slstudio/src/camera/Camera.cpp    D:/slstudio/src/camera/CameraVimba.cpp  -lC:/Program Files/Allied Vision Technologies/AVTVimba_1.4/VimbaCPP/Lib/Win64/VimbaCPP.lib -lC:/Program Files/Allied Vision Technologies/AVTVimba_1.4/VimbaC/Lib/Win64/VimbaC.lib
