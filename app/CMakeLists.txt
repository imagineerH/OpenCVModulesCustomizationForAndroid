# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.

set(CMAKE_VERBOSE_MAKEFILE on)
set(libs "${CMAKE_SOURCE_DIR}/src/main/jniLibs")

# Uncomment the following line if your source code need OpenCV headers
include_directories(/Users/zh/OpenCV-android-sdk/sdk/native/jni/include)

#------------------------------- import OpenCV modules ------------------------------------#
# Create imported target opencv_core
add_library(opencv_core STATIC IMPORTED)
set_target_properties(opencv_core PROPERTIES
  INTERFACE_LINK_LIBRARIES "$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>;$<LINK_ONLY:z>;$<LINK_ONLY:libcpufeatures>;$<LINK_ONLY:tegra_hal>"
)
set_target_properties(opencv_core PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_core.a")

# Create imported target opencv_flann
add_library(opencv_flann STATIC IMPORTED)
set_target_properties(opencv_flann PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>"
)
set_target_properties(opencv_flann PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_flann.a")

# Create imported target opencv_imgproc
add_library(opencv_imgproc STATIC IMPORTED)
set_target_properties(opencv_imgproc PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>"
)
set_target_properties(opencv_imgproc PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_imgproc.a")

# Create imported target opencv_ml
add_library(opencv_ml STATIC IMPORTED)
set_target_properties(opencv_ml PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>"
)
set_target_properties(opencv_ml PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_ml.a")

# Create imported target opencv_objdetect
add_library(opencv_objdetect STATIC IMPORTED)
set_target_properties(opencv_objdetect PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>"
)
set_target_properties(opencv_objdetect PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_objdetect.a")

# Create imported target opencv_photo
add_library(opencv_photo STATIC IMPORTED)
set_target_properties(opencv_photo PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>"
)
set_target_properties(opencv_photo PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_photo.a")

# Create imported target opencv_video
add_library(opencv_video STATIC IMPORTED)
set_target_properties(opencv_video PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>"
)
set_target_properties(opencv_video PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_video.a")

# Create imported target opencv_dnn
add_library(opencv_dnn STATIC IMPORTED)
set_target_properties(opencv_dnn PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>;$<LINK_ONLY:libprotobuf>"
)
set_target_properties(opencv_dnn PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_dnn.a")

# Create imported target opencv_imgcodecs
add_library(opencv_imgcodecs STATIC IMPORTED)
set_target_properties(opencv_imgcodecs PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>;$<LINK_ONLY:z>;$<LINK_ONLY:libjpeg>;$<LINK_ONLY:libwebp>;$<LINK_ONLY:libpng>;$<LINK_ONLY:libtiff>;$<LINK_ONLY:libjasper>;$<LINK_ONLY:IlmImf>"
)
set_target_properties(opencv_imgcodecs PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_imgcodecs.a")

# Create imported target opencv_shape
add_library(opencv_shape STATIC IMPORTED)
set_target_properties(opencv_shape PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_video;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>"
)
set_target_properties(opencv_shape PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_shape.a")

# Create imported target opencv_videoio
add_library(opencv_videoio STATIC IMPORTED)
set_target_properties(opencv_videoio PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_imgcodecs;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>"
)
set_target_properties(opencv_videoio PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_videoio.a")

# Create imported target opencv_highgui
add_library(opencv_highgui STATIC IMPORTED)
set_target_properties(opencv_highgui PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>"
)
set_target_properties(opencv_highgui PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_highgui.a")

# Create imported target opencv_superres
add_library(opencv_superres STATIC IMPORTED)
set_target_properties(opencv_superres PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_video;opencv_imgcodecs;opencv_videoio;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>"
)
set_target_properties(opencv_superres PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_superres.a")

# Create imported target opencv_features2d
add_library(opencv_features2d STATIC IMPORTED)
set_target_properties(opencv_features2d PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>"
)
set_target_properties(opencv_features2d PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_features2d.a")

# Create imported target opencv_calib3d
add_library(opencv_calib3d STATIC IMPORTED)
set_target_properties(opencv_calib3d PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_features2d;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>"
)
set_target_properties(opencv_calib3d PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_calib3d.a")

# Create imported target opencv_stitching
add_library(opencv_stitching STATIC IMPORTED)
set_target_properties(opencv_stitching PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_features2d;opencv_calib3d;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>"
)
set_target_properties(opencv_stitching PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_stitching.a")

# Create imported target opencv_videostab
add_library(opencv_videostab STATIC IMPORTED)
set_target_properties(opencv_videostab PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_photo;opencv_video;opencv_imgcodecs;opencv_videoio;opencv_highgui;opencv_features2d;opencv_calib3d;$<LINK_ONLY:dl>;$<LINK_ONLY:m>;$<LINK_ONLY:log>;$<LINK_ONLY:tbb>;$<LINK_ONLY:tegra_hal>"
)
set_target_properties(opencv_videostab PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_videostab.a")

#------------------------------- import third-party dependencies ------------------------------------#
# Create imported target libcpufeatures
add_library(libcpufeatures STATIC IMPORTED)
set_target_properties(libcpufeatures PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libcpufeatures.a")

# Create imported target libjpeg
add_library(libjpeg STATIC IMPORTED)
set_target_properties(libjpeg PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/liblibjpeg.a")

# Create imported target libtiff
add_library(libtiff STATIC IMPORTED)
set_target_properties(libtiff PROPERTIES
  INTERFACE_LINK_LIBRARIES "z"
)
set_target_properties(libtiff PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/liblibtiff.a")

# Create imported target libwebp
add_library(libwebp STATIC IMPORTED)
set_target_properties(libwebp PROPERTIES
  INTERFACE_LINK_LIBRARIES "libcpufeatures"
)
set_target_properties(libwebp PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/liblibwebp.a")

# Create imported target libjasper
add_library(libjasper STATIC IMPORTED)
set_target_properties(libjasper PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/liblibjasper.a")

# Create imported target libpng
add_library(libpng STATIC IMPORTED)
set_target_properties(libpng PROPERTIES
  INTERFACE_LINK_LIBRARIES "z"
)
set_target_properties(libpng PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/liblibpng.a")

# Create imported target IlmImf
add_library(IlmImf STATIC IMPORTED)
set_target_properties(IlmImf PROPERTIES
  INTERFACE_LINK_LIBRARIES "z"
)
set_target_properties(IlmImf PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libIlmImf.a")

# Create imported target tbb
add_library(tbb STATIC IMPORTED)
set_target_properties(tbb PROPERTIES
  INTERFACE_COMPILE_DEFINITIONS "TBB_USE_GCC_BUILTINS=1;__TBB_GCC_BUILTIN_ATOMICS_PRESENT=1"
  INTERFACE_LINK_LIBRARIES "c;m;dl"
)
set_target_properties(tbb PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libtbb.a")

# Create imported target tegra_hal
add_library(tegra_hal STATIC IMPORTED)
set_target_properties(tegra_hal PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libtegra_hal.a")

# Create imported target libprotobuf
add_library(libprotobuf STATIC IMPORTED)
set_target_properties(libprotobuf PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/liblibprotobuf.a")

#------------------------------- define library name ------------------------------------#
add_library( # Sets the name of the library.
             AITrans

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             ${CMAKE_SOURCE_DIR}/src/main/cpp/Tracking/src/color_palette.cpp
             ${CMAKE_SOURCE_DIR}/src/main/cpp/Tracking/src/flow_tracker.cpp
             ${CMAKE_SOURCE_DIR}/src/main/cpp/Tracking/src/graphics_generator.cpp
              )

# Searches for a specified prebuilt library and stores the path as a
# variable. Because CMake includes system libraries in the search path by
# default, you only need to specify the name of the public NDK library
# you want to add. CMake verifies that the library exists before
# completing its build.

find_library( # Sets the name of the path variable.
              log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log )

# Specifies libraries CMake should link to your target library. You
# can link multiple libraries, such as libraries you define in this
# build script, prebuilt third-party libraries, or system libraries.

target_link_libraries( # Specifies the target library.
                       AITrans

                       # Links the target library to the log library
                       # included in the NDK.
                       ${log-lib}

                       # OpenCV modules
                       # put the modules you want to pack into the
                       # output library inside the --whole-archive
                       # influence range

                       # core module and its corresponding dependencies
                       # will be packed with all symbols
                       "-Wl,--whole-archive"

                       "-Wl,--no-whole-archive"

                       # the following modules will not be packed
                       opencv_core
                       opencv_flann
                       opencv_imgproc
                       opencv_ml
                       opencv_objdetect
                       opencv_photo
                       opencv_video
                       opencv_dnn
                       opencv_imgcodecs
                       opencv_shape
                       opencv_videoio
                       opencv_highgui
                       opencv_superres
                       opencv_features2d
                       opencv_calib3d
                       opencv_stitching
                       opencv_videostab

                       # don't need link third-party dependencies directly,
                       # they will be linked into target library according
                       # to OpenCV modules which are used

                       # project-level use
                       jnigraphics
                       android
                       log
                        )