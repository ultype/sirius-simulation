#ifndef PUBLIC_MODIFIED_DATA_SIRIUS_UTILITY_H_
#define PUBLIC_MODIFIED_DATA_SIRIUS_UTILITY_H_


#define IS_MISSION_ARRIVED(mission, bitmap, code) ((bitmap) & (0x1U << (mission)) && (code) == (mission))
#endif  // PUBLIC_MODIFIED_DATA_SIRIUS_UTILITY_H_
