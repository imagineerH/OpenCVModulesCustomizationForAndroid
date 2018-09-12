# OpenCV按模块自定义组合打包（Android so库）

## 概述

OpenCV是一个跨平台的计算机视觉库，可用于开发实时的图像识别、计算机视觉以及模式识别程序。OpenCV Android sdk中提供了官方so库libopencv_java3.so，集成了OpenCV的所有模块，但体积较大，原始so大小为11M（以armeabi-v7a为例，下同），集成到APK中带来的体积增量也在5M以上。实际使用中，可能只需用到OpenCV的部分模块，若只将需要的模块构建成so库，则库体积能缩小很多。本打包工程基于CMake，能灵活配置所需的OpenCV模块集成Android so库。

## 工程及版本信息

- OpenCV版本：3.4.1
- CMake版本：3.6.4111459（Android sdk自带）
- Android studio版本：3.1.4
- Android ndk版本：r15c

## 使用

在app/CMakeLists.txt的target_link_libraries部分，可自定义需要组合构建的模块，默认只引入了core模块

```
target_link_libraries( # Specifies the target library.
                       opencv_java3

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
                       opencv_core
                       "-Wl,--no-whole-archive"

                       # the following modules will not be packed
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
                        )
```

打包时，选择“Build apk”或执行assemble命令，输出的so库放在app/output-so-file路径下，您也可以在build.gradle自定义输出路径

```groovy
 externalNativeBuild {
            cmake {
                arguments "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=${projectDir}/output-so-file"
                ......
            }
        }
```

## 其他说明

1. 若需自定义适配的ABI，或配置其他CMake参数，和普通CMake工程无异，在build.gradle或CMakeLists.txt中修改或添加即可
2. OpenCV各模块静态链接库和第三方静态库的相互依赖关系参考app/OpenCVModules.cmake，该文件原始路径为OpenCV-android-sdk/sdk/native/jni/abi-armeabi-v7a
3. 若需改变OpenCV版本，您可自行下载所需版本的OpenCV Android sdk，将其中sdk/native/staticlibs/${target ABI}及sdk/native/thirdparty/libs/${target ABI}中的所有静态库拷贝至app/src/main/jniLibs/${target ABI}中进行替换，各静态库间依赖关系也以该版本sdk中的OpenCVModules.cmake为准













