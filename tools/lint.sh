#!/bin/bash

# User Guide:
# $ ./lint.sh [emacs|eclipse|vs7|junit]
# If the first parameter is unset, the default value would be "emacs".

output=${1:-emacs}
# Find files with Emacs Regular Expression and exclude submodule directories.
files=$( find ./ -iregex "\./models/\(cad\|dm\|gnc\|math\|sensor\)/.*" -prune -o -iregex ".*\.\(h\|hh\|c\|cc\|cpp\)" -print \
| grep -vE \QA_Tool)
cpplint \
  --counting=detailed \
  --extensions=h,hh,c,cc,cpp \
  --filter=-build/include,-legal/copyright,-runtime/references \
  --linelength=1024 \
  --output=${output} \
  --verbose=0 \
  ${files} \
