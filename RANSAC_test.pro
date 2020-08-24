TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    LineFittingSample.cpp \
    TriangulationRansacSample.cpp

HEADERS += \
    AbstractModel.hpp \
    GRANSAC.hpp \
    LineModel.hpp \
    TriangulationModel.hpp



INCLUDEPATH += /usr/local/include \
                /usr/lib/gcc/x86_64-linux-gnu/6/include \
                /usr/local/include/opencv4 \
                /usr/local/include/opencv4/opencv2 \
                /home/shuai/Software/eigen-3.2.10/Eigen \
                /opt/intel/compilers_and_libraries_2019.5.281/linux/mkl/include


LIBS +=  /usr/local/lib/libopencv_highgui.so \
            /usr/local/lib/libopencv_core.so    \
            /usr/local/lib/libopencv_imgproc.so   \
            /usr/local/lib/libopencv_videoio.so   \
            /usr/local/lib/libopencv_calib3d.so \
           /usr/local/lib/libopencv_imgcodecs.so \
            /opt/intel/compilers_and_libraries_2019.5.281/linux/mkl/lib/intel64_lin/libmkl_core.so \
            /opt/intel/compilers_and_libraries_2019.5.281/linux/mkl/lib/intel64_lin/libmkl_intel_lp64.so \
            /opt/intel/compilers_and_libraries_2019.5.281/linux/mkl/lib/intel64_lin/libmkl_intel_thread.so

LIBS += -liomp5 -lpthread -lm -lstdc++fs -L/opt/intel/mkl/lib/intel64 -L/opt/intel/lib/intel64
