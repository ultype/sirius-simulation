#ifndef EXE_XIL_COMMON_INCLUDE_SIRIUS_UTILITY_H_
#define EXE_XIL_COMMON_INCLUDE_SIRIUS_UTILITY_H_


#define IS_MISSION_ARRIVED(mission, bitmap, code) ((bitmap) & (0x1U << (mission)) && (code) == (mission))

#define PRINT_MISSION_MESSAGE(side, sim_time, text, code) do { \
            fprintf(stderr, "[%s:%s:%f] %s: %d\n" , side, __FUNCTION__, sim_time, text, code);} while (0)
#endif  // EXE_XIL_COMMON_INCLUDE_SIRIUS_UTILITY_H_
