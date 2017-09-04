include_directories(.)
rosbuild_add_library(VMCLIB CCommunicationLayer.cpp  CTranslationLayer.cpp  CvmcAPI.cpp  CVmc.cpp)
