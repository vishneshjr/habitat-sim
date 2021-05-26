#!/bin/bash
# export LD_PRELOAD="$(gcc -print-file-name=libasan.so):$LD_PRELOAD"
export CC=clang
export CXX=clang++
export MAGNUM_GPU_VALIDATION=ON
export MAGNUM_LOG=verbose
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/nvidia-opengl/:$LD_LIBRARY_PATH
#C_COMPILER_FLAGS="-fsanitize=address -fno-omit-frame-pointer -fno-optimize-sibling-calls -g -O1"
C_COMPILER_FLAGS="-fsanitize=leak -fno-omit-frame-pointer -g -O1"
export CXXFLAGS="${C_COMPILER_FLAGS}"
export CFLAGS="${C_COMPILER_FLAGS}"
GIT_ROOT_DIR="$(git rev-parse --show-toplevel)"
export ASAN_OPTIONS="fast_unwind_on_malloc=0:suppressions=${GIT_ROOT_DIR}/tools/asan_suppressions.txt"
export LSAN_OPTIONS="suppressions=${GIT_ROOT_DIR}/tools/lsan_suppressions.txt"
export CORRADE_TEST_COLOR=ON
export GTEST_COLOR=yes
#build/tests/SimTest
"${GIT_ROOT_DIR}/build.sh" --headless --bullet --build-datatool  --debug --run-tests --cmake --cmake-args="-Dgtest_disable_pthreads=ON -DRAPIDJSON_BUILD_ASAN=ON"
#'cmake' "-H$(pwd)/src" '-Bbuild' '-DBUILD_PYTHON_BINDINGS=OFF' "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=$(pwd)/habitat_sim/_ext" '-DPYTHON_EXECUTABLE=/Users/agokaslan/venv/ai_habitat/bin/python' -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -Dgtest_disable_pthreads=ON -DCMAKE_BUILD_TYPE=Debug -DBUILD_GUI_VIEWERS=OFF -DBUILD_TEST=ON -DBUILD_WITH_BULLET=ON -DBUILD_DATATOOL=ON -DRAPIDJSON_BUILD_ASAN=ON
#cd build || exit
#make -j 4
#ctest -V
#TODO Add Python Tests for ASAN
