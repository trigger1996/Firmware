############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

set(OPTFLOW_SRCS
	optflow/optflow_task.cpp
	optflow/farneback_simplified.cpp
)

set(SRCS
	lidar_driver.cpp
	slam.cpp
	${OPTFLOW_SRCS}
	)

set(RPLIDAR_SRCS
	sdk/src/rplidar_driver.cpp
	sdk/src/arch/linux/net_serial.cpp
	sdk/src/arch/linux/thread.hpp
	sdk/src/arch/linux/timer.cpp
	sdk/src/hal/thread.cpp
)

set(RPLIDAR_INCLUDE_DIR
	sdk/include/
	sdk/src/
	sdk/arch/linux/
	sdk/src/hal/
)

set(OPENCV_DIR
        #/home/ghost/src/opencv/opencv-3.1.0/rpi_install
        /usr/local
        )

include_directories(
	/
	${OPENCV_DIR}/include/
	${RPLIDAR_INCLUDE_DIR}
	optflow/
	)

link_libraries(
        ${OPENCV_DIR}/lib/libopencv_calib3d.so
        ${OPENCV_DIR}/lib/libopencv_core.so
        ${OPENCV_DIR}/lib/libopencv_features2d.so
        ${OPENCV_DIR}/lib/libopencv_flann.so
        ${OPENCV_DIR}/lib/libopencv_highgui.so
        # ${OPENCV_DIR}/lib/libopencv_imgcodecs.so
        ${OPENCV_DIR}/lib/libopencv_imgproc.so
        ${OPENCV_DIR}/lib/libopencv_ml.so
        ${OPENCV_DIR}/lib/libopencv_objdetect.so
        ${OPENCV_DIR}/lib/libopencv_photo.so
        # ${OPENCV_DIR}/lib/libopencv_shape.so
        ${OPENCV_DIR}/lib/libopencv_stitching.so
        ${OPENCV_DIR}/lib/libopencv_superres.so
        ${OPENCV_DIR}/lib/libopencv_ts.a
        ${OPENCV_DIR}/lib/libopencv_video.so
        # ${OPENCV_DIR}/lib/libopencv_videoio.so
        ${OPENCV_DIR}/lib/libopencv_videostab.so
        )


px4_add_module(
	MODULE modules__rplidar
	MAIN rplidar
	STACK_MAIN 32000
	SRCS
		rplidar_task_main.cpp
		${RPLIDAR_SRCS}
		${SRCS}
	DEPENDS
		platforms__common
	)


# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
