#!/bin/sh
# lora.cpp(프로토콜 스택)를 호스트에서 목(mock) Arduino/RadioLib/FreeRTOS 위에 올려 돌린다.
# 하드웨어 없이 청크/escape, src별 재조립, idle 만료, 봉투 검증, PING 주소지정, §8a 양보, LBT를 검증.
# 사용: tools/hosttest/run.sh            (pager-vehicle의 config.h = P01 기준)
set -e
cd "$(dirname "$0")"
c++ -std=c++17 -Wno-deprecated-volatile -I. -I../../pager-vehicle -o ./hosttest.bin test.cpp
./hosttest.bin
