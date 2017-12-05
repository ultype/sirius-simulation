#ifndef MODELS_ICF_INCLUDE_RS422_SERIALPORT_H_
#define MODELS_ICF_INCLUDE_RS422_SERIALPORT_H_

#include "icf_export.h"

#define SERIAL_PORT(idx) "/dev/ttyAP"#idx
#define RS422_SERIAL_BUFFER_SIZE 1024
#define USER_BAUD_RATE (B921600)
#define RS422_HEADER_SIZE sizeof(struct rs422_frame_header_t)

extern const char *que_port_map[8];

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
    uint8_t qidx;
    struct rs422_frame_header_t frame;
    uint32_t payload_size;
    void *payload;
};

struct rs422_device_name_t {
    char portname[IFNAMSIZ];
};

#ifdef __cplusplus
extern "C" {
#endif
int rs422_devinfo_init(struct rs422_device_info_t *dev_info,
                       const char *portname,
                       void *payload,
                       uint32_t payload_size,
                       uint8_t qidx);
int rs422_serialport_init(struct rs422_device_info_t *rs422_dev);
int open_port(char *portname);
int set_interface_attribs(int fd, int speed, int parity);
int32_t rs422_data_send_scatter(int fd, uint8_t *payload, uint32_t frame_len);
uint8_t* rs422_frame_alloc(uint32_t size);
int rs422_frame_header_set(struct rs422_frame_header_t *frame, const uint8_t *payload, const uint32_t data_len);
int rs422_frame_payload_copy(uint8_t *out_buff, const uint8_t *payload, const uint32_t data_len);
int rs422_frame_header_copy(uint8_t *out_buff, struct rs422_frame_header_t *frame);

#ifdef __cplusplus
}
#endif
#endif   //  MODELS_ICF_INCLUDE_RS422_SERIALPORT_H_
