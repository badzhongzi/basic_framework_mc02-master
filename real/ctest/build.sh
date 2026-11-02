#!/bin/sh
# 编译并运行 C 侧 float32 数值回归（QP + RNEA + tick 循环）。
# 用法（在本目录下）:  sh build.sh
# 先跑 dump_qp.py、dump_rnea.py、dump_loop.py 生成用例文件。
set -e

# 编译一个测试。失败必须中止：以前用 `gcc ... | grep -v ... || true` 过滤警告，
# 管道把 gcc 的退出码吞掉了，编译失败时会悄悄拿上一次的旧 exe 去跑测试。
# -Werror=implicit-function-declaration：隐式声明返回 int，对 float 函数是静默截断。
compile() {
    out="$1"; shift
    rm -f "$out" "$out.exe"
    log="$BUILD/$(basename "$out").log"
    if ! gcc -O2 -std=c11 -Wall -Werror=implicit-function-declaration \
             $INC "$@" -lm -o "$out" > "$log" 2>&1; then
        cat "$log"
        echo "!!! 编译失败: $out"
        exit 1
    fi
    # 初始化式写错（花括号层数、元素个数）gcc 只报一个没有开关名的 warning，
    # 结果却是错的常数表——观测器矩阵就因此被悄悄写坏过一次。所以单独拦下来。
    if grep -E "excess elements in|braces around scalar initializer" "$log"; then
        echo "!!! 生成的常数表初始化式有误: $out"
        exit 1
    fi
    # CasADi 生成的代码有 unused 警告，不是我们的代码，不显示
    grep -v "defined but not used\|unused-function\|^ *|\|^ *[0-9]* |\|In function\|^ *~\|^ *\^" "$log" | grep "warning" || true
}

HERE=$(cd "$(dirname "$0")" && pwd)
GEN="$HERE/../generated"
BUILD="$HERE/build"
mkdir -p "$BUILD"

INC="-I $GEN"

# P7 之后求解器是 mpc_6axis.c 自己的定迭代 ADMM，不再依赖 OSQP。
# mpc_inertia_ratio() 调 crba()，所以 crba_6axis.c 必须一起链接。
QP_SRC="$GEN/mpc_6axis.c $GEN/crba_6axis.c"

echo "=== 编译 test_qp ==="
compile "$BUILD/test_qp" "$HERE/test_qp.c" $QP_SRC

echo "=== 编译 test_rnea ==="
compile "$BUILD/test_rnea" "$HERE/test_rnea.c" "$GEN/rnea_6axis.c"

# tick 循环回归要同时链接 RNEA 和整套 QP
echo "=== 编译 test_loop ==="
compile "$BUILD/test_loop" "$HERE/test_loop.c" "$GEN/rnea_6axis.c" $QP_SRC

echo
echo "########## 求解器回归（单次求解，含约束起作用的场景）##########"
"$BUILD/test_qp" "$HERE/qp_cases.txt"
echo
echo "########## RNEA 回归 ##########"
"$BUILD/test_rnea" "$HERE/rnea_cases.txt"
echo
echo "########## tick 循环回归（每拍全解+约束+装配）##########"
"$BUILD/test_loop" "$HERE/loop_cases.txt"
