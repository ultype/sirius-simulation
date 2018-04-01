#ifndef EXE_XIL_COMMON_INCLUDE_SIRIUS_UTILITY_H_
#define EXE_XIL_COMMON_INCLUDE_SIRIUS_UTILITY_H_


#define IS_MISSION_ARRIVED(mission, bitmap, code) ((bitmap) & (0x1U << (mission)) && (code) == (mission))
#endif  // EXE_XIL_COMMON_INCLUDE_SIRIUS_UTILITY_H_
