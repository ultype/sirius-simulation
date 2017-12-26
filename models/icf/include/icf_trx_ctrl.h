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

#define ICF_CTRLBLK_MAXQUEUE_NUMBER  16

typedef enum _ENUM_ICF_SW_QUEUE {
    EMPTY_SW_QIDX = -1,
    TVC_SW_QIDX = 0,
    IMU01_SW_QIDX = 1,
    RATETBL_X_SW_QIDX = 2,
    RATETBL_Y_SW_QIDX = 3,
    RATETBL_Z_SW_QIDX = 4,
    IMU02_SW_QIDX = 5,
    GPSR01_SW_QIDX = 6,
    GPSR02_SW_QIDX = 7,
    FLIGHT_COMPUTER_SW_QIDX = 8,
    GPSRF_EMULATOR_SW_QIDX = 9,
    NUM_OF_SW_QUE
}ENUM_ICF_SW_QUEUE;

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
}ENUM_ICF_DRIVERS_ID;


struct icf_mapping {
    int sw_queue;
    int hw_port;
    int driver_id;
};


struct icf_ctrl_port {
    int hw_port;
    char ifname[IFNAMSIZ];
    int netport;
    uint8_t dev_type;
    void *drv_priv_data;
    struct icf_driver_ops *drv_priv_ops;
};

struct icf_ctrl_queue {
    int queue_idx;
    uint8_t direction;
    struct icf_ctrl_port *port; 
    struct ringbuffer_t data_ring;
};


struct icf_ctrlblk_t {
    struct icf_ctrl_queue *ctrlqueue[ICF_CTRLBLK_MAXQUEUE_NUMBER];
};

#ifdef __cplusplus
extern "C" {
#endif

void *icf_alloc_mem(size_t size);
void icf_free_mem(void *ptr);
int icf_ctrlblk_init(struct icf_ctrlblk_t* C);
int icf_ctrlblk_deinit(struct icf_ctrlblk_t* C);
int icf_rx_ctrl_job(struct icf_ctrlblk_t* C, int pidx);

int icf_tx_direct(struct icf_ctrlblk_t* C, int qidx, void *payload, uint32_t size);
int icf_tx_send2ring(struct icf_ctrlblk_t* C, int qidx, void *payload, uint32_t size);
int icf_tx_ctrl_job(struct icf_ctrlblk_t* C, int qidx);


#if 0
int icf_eth_tx_direct(struct icf_tx_ctrl_t* C, void *payload, uint32_t size);
int icf_eth_tx_enqueue2ring(struct icf_tx_ctrl_t* C, void *payload, uint32_t size);
int icf_eth_tx_ctrl_job(struct icf_tx_ctrl_t* C);
#endif
#ifdef __cplusplus
}
#endif

#endif  // MODELS_ICF_INCLUDE_ICF_TRX_CTRL_H_
