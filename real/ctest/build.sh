#!/bin/sh
# 编译并运行 C 侧 float32 数值回归（QP + RNEA）。
# 用法（在本目录下）:  sh build.sh
# 先跑 dump_qp.py 和 dump_rnea.py 生成用例文件。
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
GEN="$HERE/../generated"
BUILD="$HERE/build"
mkdir -p "$BUILD"

INC="-I $GEN -I $GEN/generated_solver/common
     -I $GEN/generated_solver/common/inc/public
     -I $GEN/generated_solver/common/inc/private"

echo "=== 编译 test_qp ==="
gcc -O2 -std=c11 -Wall $INC \
    "$HERE/test_qp.c" \
    "$GEN/mpc_6axis.c" \
    "$GEN"/generated_solver/common/src/*.c \
    "$GEN"/generated_solver/joint0/mpc_j0workspace.c \
    "$GEN"/generated_solver/joint1/mpc_j1workspace.c \
    "$GEN"/generated_solver/joint2/mpc_j2workspace.c \
    "$GEN"/generated_solver/joint3/mpc_j3workspace.c \
    "$GEN"/generated_solver/joint4/mpc_j4workspace.c \
    "$GEN"/generated_solver/joint5/mpc_j5workspace.c \
    -lm -o "$BUILD/test_qp" 2>&1 | grep -v "defined but not used" || true

echo "=== 编译 test_rnea ==="
gcc -O2 -std=c11 -Wall $INC \
    "$HERE/test_rnea.c" "$GEN/rnea_6axis.c" \
    -lm -o "$BUILD/test_rnea"

echo
echo "########## QP 回归 ##########"
"$BUILD/test_qp" "$HERE/qp_cases.txt"
echo
echo "########## RNEA 回归 ##########"
"$BUILD/test_rnea" "$HERE/rnea_cases.txt"
