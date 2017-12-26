#ifndef MODELS_ICF_INCLUDE_RS422_SERIALPORT_H_
#define MODELS_ICF_INCLUDE_RS422_SERIALPORT_H_
/********************************* TRICK HEADER *******************************
PURPOSE:
      RX Ctrl
LIBRARY DEPENDENCY:
      (
      	(../src/icf_utility.c)
      	(../src/rs422_serialport.c)
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/
#include "icf_export.h"
#include "icf_utility.h"
#include "icf_drivers.h"
#define SERIAL_PORT(idx) "/dev/ttyAP"#idx
#define RS422_SERIAL_BUFFER_SIZE 1024
#define USER_BAUD_RATE (B921600)
#define RS422_HEADER_SIZE sizeof(struct rs422_frame_header_t)

#define SERIAL_PORT_ENABLE_BITMAP (0x1)// (0x7F)
#define SERIAL_PORT_IS_ENABLE(qidx) ((SERIAL_PORT_ENABLE_BITMAP >> qidx) & 0x1)

typedef enum _ENUM_HW_RS422_TX_QUE_T {
    IMU_01 = 0,
    RATE_TABLE_X,
    RATE_TABLE_Y,
    RATE_TABLE_Z,
    IMU_02,
    GPSR_01,
    GPSR_02,
    RS422_TXQ_NUM
} ENUM_HW_RS422_TX_QUE_T;


struct rs422_frame_header_t {
    uint32_t payload_len;
    uint32_t crc;
    uint32_t seq_no;
} __attribute__((packed));



struct rs422_device_info_t {
    char portname[IFNAMSIZ];
    int32_t rs422_fd;
    struct rs422_frame_header_t frame;
    uint32_t header_size;
};

struct rs422_device_name_t {
    char portname[IFNAMSIZ];
};

#ifdef __cplusplus
extern "C" {
#endif
int rs422_serialport_init(void **priv_data, char *ifname, int netport);
int rs422_serialport_deinit(void **priv_data);
int open_port(char *portname);
int set_interface_attribs(int fd, int speed, int parity);

#ifdef __cplusplus
}
#endif
#endif   //  MODELS_ICF_INCLUDE_RS422_SERIALPORT_H_
