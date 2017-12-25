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

typedef enum _ENUM_ICF_DEVICE_QIDX {
    EMPTY_DEVICE_QIDX = -1,
    TVC_DEVICE_QIDX = 0,
    RCS_DEVICE_QIDX = 1,
    VALAVE_DEVICE_QIDX = 2,
    IMU01_DEVICE_QIDX = 3,
    RATETBL_X_DEVICE_QIDX = 4,
    RATETBL_Y_DEVICE_QIDX = 5,
    RATETBL_Z_DEVICE_QIDX = 6,
    IMU02_DEVICE_QIDX = 7,
    GPSR01_DEVICE_QIDX = 8,
    GPSR02_DEVICE_QIDX = 9,
    GPSRF_EMULATOR_DEVICE_QIDX = 10,
    FLIGHT_COMPUTER_DEVICE_QIDX = 11,
    NUM_OF_DEVICE_QUE
}ENUM_ICF_DEVICE_QIDX;

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


struct icf_ctrl_port {
    char ifname[IFNAMSIZ];
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
int icf_rx_ctrl_job(struct icf_ctrlblk_t* C, int qidx);
#if 0
int icf_tx_direct(struct icf_tx_ctrl_t* C, ENUM_HW_RS422_TX_QUE_T qidx, void *payload, uint32_t size);
int icf_tx_send2ring(struct icf_tx_ctrl_t* C, ENUM_HW_RS422_TX_QUE_T qidx, void *payload, uint32_t size);
int icf_tx_ctrl_job(struct icf_tx_ctrl_t* C, ENUM_HW_RS422_TX_QUE_T qidx);



int icf_eth_tx_direct(struct icf_tx_ctrl_t* C, void *payload, uint32_t size);
int icf_eth_tx_enqueue2ring(struct icf_tx_ctrl_t* C, void *payload, uint32_t size);
int icf_eth_tx_ctrl_job(struct icf_tx_ctrl_t* C);
#endif
#ifdef __cplusplus
}
#endif

#endif  // MODELS_ICF_INCLUDE_ICF_TRX_CTRL_H_
