#ifndef EXE_XIL_COMMON_INCLUDE_SIRIUS_UTILITY_H_
#define EXE_XIL_COMMON_INCLUDE_SIRIUS_UTILITY_H_


#define IS_FLIGHT_EVENT_ARRIVED(event, bitmap, code) ((bitmap) & (0x1U << (event)) && (code) == (event))

#define PRINT_FLIGHT_EVENT_MESSAGE(side, sim_time, text, code) do { \
            fprintf(stderr, "[%s:%s:%f] %s: %d\n" , side, __FUNCTION__, sim_time, text, code);} while (0)
#endif  // EXE_XIL_COMMON_INCLUDE_SIRIUS_UTILITY_H_
