#!/bin/sh
# 在工程根目录运行: sh tests/master_link/run_host_test.sh
# 用电脑上的 gcc 编译协议部分和测试, 不需要板子.
set -e
OUT=build_host_test
mkdir -p "$OUT"
gcc -std=c11 -Wall -Wextra -O2 -DMASTER_LINK_HOST_TEST \
    -Itests/master_link/host_stub -Imodules/master_link -Imodules/master_machine -Imodules/algorithm \
    tests/master_link/test_master_link_host.c \
    modules/master_link/master_link_protocol.c \
    modules/master_machine/seasky_protocol.c \
    modules/algorithm/crc8.c modules/algorithm/crc16.c \
    -lm -o "$OUT/test_master_link_host"
"./$OUT/test_master_link_host"
