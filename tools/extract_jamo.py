#!/usr/bin/env python3
"""
GNU Unifont .hex 파일에서 한글 호환 자모(U+3131..U+3163) 51자의
8x16 비트맵을 뽑아 jamo_font.cpp의 배열 초기화 부분에 그대로 붙일
수 있는 C 코드 조각을 stdout으로 출력.

준비:
  1. https://unifoundry.com/pub/unifont/unifont-15.1.05/font-builds/
     에서 unifont-15.1.05.hex.gz 받아서 압축 풀기
     (또는 brew/apt로 unifont 패키지 설치 후 unifont.hex 찾기)
  2. python3 tools/extract_jamo.py unifont-15.1.05.hex > out.txt
  3. out.txt 내용을 jamo_font.cpp의 JAMO_FONT 배열에 복사

Unifont 포맷:
  각 줄: "XXXX:HHHH...HHHH"
    XXXX     = 16진 codepoint
    HHHH...  = 비트맵 hex. 32 hex digits면 8픽셀폭, 64면 16픽셀폭.
  자모는 보통 16픽셀폭(64 digits)이라 8픽셀로 축소 필요 (왼쪽 8픽셀 사용).

  각 byte는 MSB-first per byte (= bit 7 = leftmost pixel) → drawBitmap과 호환.
"""

import sys

if len(sys.argv) != 2:
    print(__doc__, file=sys.stderr)
    sys.exit(1)

CODEPOINTS = list(range(0x3131, 0x3164))   # U+3131..U+3163

bitmaps = {}
with open(sys.argv[1], 'r') as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        try:
            cp_str, hex_data = line.split(':')
        except ValueError:
            continue
        cp = int(cp_str, 16)
        if cp in CODEPOINTS:
            bitmaps[cp] = hex_data

JAMO_NAMES = "ㄱㄲㄳㄴㄵㄶㄷㄸㄹㄺㄻㄼㄽㄾㄿㅀㅁㅂㅃㅄㅅㅆㅇㅈㅉㅊㅋㅌㅍㅎㅏㅐㅑㅒㅓㅔㅕㅖㅗㅘㅙㅚㅛㅜㅝㅞㅟㅠㅡㅢㅣ"

for i, cp in enumerate(CODEPOINTS):
    hex_data = bitmaps.get(cp)
    if hex_data is None:
        bytes32 = [0] * 32
    elif len(hex_data) == 32:
        # 8x16 → 16x16으로 패딩 (오른쪽 8픽셀에 0 추가)
        rows = [hex_data[j:j+2] for j in range(0, 32, 2)]
        bytes32 = []
        for r in rows:
            bytes32.append(int(r, 16))
            bytes32.append(0)
    elif len(hex_data) == 64:
        # 16x16: row당 2 byte 그대로
        bytes32 = [int(hex_data[j:j+2], 16) for j in range(0, 64, 2)]
    else:
        bytes32 = [0] * 32

    bytes_str = ', '.join(f'0x{b:02X}' for b in bytes32)
    name = JAMO_NAMES[i] if i < len(JAMO_NAMES) else '?'
    print(f'  /* {cp:04X} {name} */ {{ {bytes_str} }},')
