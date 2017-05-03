#!/bin/bash

# User Guide:
# $ ./lint.sh [emacs|eclipse|vs7|junit]
# If the first parameter is unset, the default value would be "emacs".

output=${1:-emacs}
cpplint \
  --counting=detailed \
  --extensions=h,hh,c,cc,cpp \
  --filter=-build/include,-legal/copyright \
  --linelength=1024 \
  --output=${output} \
  --recursive \
  --verbose=0 \
./
