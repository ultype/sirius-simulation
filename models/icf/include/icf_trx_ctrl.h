#ifndef MODELS_ICF_INCLUDE_ICF_TRX_CTRL_H_
#define MODELS_ICF_INCLUDE_ICF_TRX_CTRL_H_
/********************************* TRICK HEADER *******************************
PURPOSE:
      TRX Ctrl
LIBRARY DEPENDENCY:
      (
      	(../src/icf_utility.c)
      	(../src/socket_can.c)
      	(../src/icf_trx_ctrl.c)
        (../src/ringbuffer.c)
        (../src/rs422_serialport.c)
        (../src/ethernet.c)
        (../src/icf_drivers.c)
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/

#include "icf_utility.h"
#include "ringbuffer.h"
#include "icf_drivers.h"
#include "dsp_can_interfaces.h"
#include "trick/exec_proto.h"
#define ICF_CTRLBLK_MAXQUEUE_NUMBER  16
#define ICF_CTRLBLK_MAXPORT_NUMBER  16
#define ICF_EGSE_CONNECT_IP "127.0.0.1"
#ifdef CONFIG_HIL_ENABLE
#define HIL_INTF 1
#else
#define HIL_INTF 0
#endif  //  CONFIG_HIL_ENABLE

typedef enum _ENUM_ICF_STATUS {
    ICF_STATUS_FAIL = -1,
    ICF_STATUS_SUCCESS = 0
}ENUM_ICF_STATUS;

typedef enum _ENUM_ICF_EGSE_SW_QUEUE {
    EGSE_EMPTY_SW_QIDX = -1,
    EGSE_TVC_SW_QIDX = 0,
    EGSE_IMU01_SW_QIDX = 1,
    EGSE_RATETBL_X_SW_QIDX = 2,
    EGSE_RATETBL_Y_SW_QIDX = 3,
    EGSE_RATETBL_Z_SW_QIDX = 4,
    EGSE_IMU02_SW_QIDX = 5,
    EGSE_GPSR01_SW_QIDX = 6,
    EGSE_GPSR02_SW_QIDX = 7,
    EGSE_FLIGHT_COMPUTER_SW_QIDX = 8,
    EGSE_GPSRF_EMULATOR_SW_QIDX = 9,
    EGSE_SIL_DOWNLINK_SW_QIDX = 10,
    EGSE_RX_MISSION_EVENT_QIDX = 11,
    EGSE_TX_GPSRF_EMU_QIDX = 12,
    EGSE_NUM_OF_SW_QUE
}ENUM_ICF_EGSE_SW_QUEUE;

typedef enum _ENUM_ICF_ESPS_SW_QUEUE {
    ESPS_EMPTY_SW_QIDX = -1,
    ESPS_TVC_SW_QIDX = 0,
    ESPS_GNC_SW_QIDX = 1,
    ESPS_TX_MISSION_CODE_QIDX = 2,
    ESPS_NUM_OF_SW_QUE
}ENUM_ICF_ESPS_SW_QUEUE;

typedef enum _ENUM_ICF_HW_PORT {
    EMPTY_HW_PORT = -1,
    HW_PORT0 = 0,
    HW_PORT1 = 1,
    HW_PORT2 = 2,
    HW_PORT3 = 3,
    HW_PORT4 = 4,
    HW_PORT5 = 5,
    HW_PORT6 = 6,
    HW_PORT7 = 7,
    HW_PORT8 = 8,
    HW_PORT9 = 9,
    NUM_OF_HW_PORT
}ENUM_ICF_HW_PORT;

typedef enum _ENUM_ICF_NETPORT {
    EMPTY_NETPORT = -1,
    NUM_OF_NETPORT
}ENUM_ICF_NETPORT;

typedef enum _ENUM_ICF_DEVICE_TYPE {
    NONE_DEVICE_TYPE = 0x0,
    CAN_DEVICE_TYPE = 0x1,
    RS422_DEVICE_TYPE = 0x2,
    ETHERNET_DEVICE_TYPE = 0x3,
    NUM_OF_DEVICE_TYPE
}ENUM_ICF_DEVICE_TYPE;

typedef enum _ENUM_ICF_DIRECTION {
    ICF_DIRECTION_RX,
    ICF_DIRECTION_TX
}ENUM_ICF_DIRECTION;

typedef enum _ENUM_ICF_DRIVERS_ID {
    ICF_DRIVERS_ID0 = 0,
    ICF_DRIVERS_ID1,
    ICF_DRIVERS_ID2,
    ICF_DRIVERS_ID3,
}ENUM_ICF_DRIVERS_ID;

typedef enum _ENUM_ICF_SYSTEM_TYPE {
    ICF_SYSTEM_TYPE_EGSE = 0,
    ICF_SYSTEM_TYPE_ESPS = 1,
    ICF_SYSTEM_TYPE_SIL_EGSE = 2,
    ICF_SYSTEM_TYPE_SIL_ESPS = 3,
}ENUM_ICF_SYSTEM_TYPE;

struct icf_mapping {
    int hw_port_idx;
    int sw_queue;
    int driver_id;
};


struct icf_ctrl_port {
    uint8_t enable;
    int hw_port_idx;
    char ifname[IFNAMSIZ];
    int netport;
    uint8_t dev_type;
    void *drv_priv_data;
    struct icf_driver_ops *drv_priv_ops;
};

struct icf_ctrl_queue {
    uint8_t enable;
    int queue_idx;
    uint8_t direction;
    struct icf_ctrl_port *port;
    struct ringbuffer_t data_ring;
};


struct icf_ctrlblk_t {
    int system_type;
    struct icf_ctrl_queue *ctrlqueue[ICF_CTRLBLK_MAXQUEUE_NUMBER];
    struct icf_ctrl_port *ctrlport[ICF_CTRLBLK_MAXPORT_NUMBER];
};

#ifdef __cplusplus
extern "C" {
#endif

void *icf_alloc_mem(size_t size);
void icf_free_mem(void *ptr);
int icf_ctrlblk_init(struct icf_ctrlblk_t* C, int system_type);
int icf_ctrlblk_deinit(struct icf_ctrlblk_t* C, int system_type);
int icf_rx_dequeue(struct icf_ctrlblk_t* C, int qidx, void *payload, uint32_t size);
int icf_rx_ctrl_job(struct icf_ctrlblk_t* C, int pidx, int rx_buff_size);

int icf_tx_direct(struct icf_ctrlblk_t* C, int qidx, void *payload, uint32_t size);
int icf_tx_enqueue(struct icf_ctrlblk_t* C, int qidx, void *payload, uint32_t size);
int icf_tx_ctrl_job(struct icf_ctrlblk_t* C, int qidx);
void icf_heartbeat(void);

#ifdef __cplusplus
}
#endif

#endif  // MODELS_ICF_INCLUDE_ICF_TRX_CTRL_H_
