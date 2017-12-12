#!/bin/bash

# User Guide:
# $ ./lint.sh [emacs|eclipse|vs7|junit]
# If the first parameter is unset, the default value would be "emacs".
# It would generate the files "C_style_report" and "Cpp_style_report".

output=${1:-emacs}
# Find files with Emacs Regular Expression and exclude submodule directories.
C_files=$( find ./ -iregex "\./models/\(cad\|dm\|gnc\|math\|sensor\)/.*" -prune -o -iregex ".*\.\(h\|c\)" -print \
| grep -vE \QA_Tool)
Cpp_files=$( find ./ -iregex "\./models/\(cad\|dm\|gnc\|math\|sensor\)/.*" -prune -o -iregex ".*\.\(hh\|cc\|cpp\)" -print \
| grep -vE \QA_Tool)

# C files checking
cpplint \
  --counting=detailed \
  --extensions=h,c \
  --filter=-build/include,-legal/copyright,-readability/casting,-runtime/references \
  --linelength=1024 \
  --output=${output} \
  --verbose=0 \
  ${C_files} \
2> >(tee C_style_report)

# C++ files checking
cpplint \
  --counting=detailed \
  --extensions=hh,cc,cpp \
  --filter=-build/include,-legal/copyright,-runtime/references \
  --linelength=1024 \
  --output=${output} \
  --verbose=0 \
  ${Cpp_files} \
2> >(tee Cpp_style_report)
