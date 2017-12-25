#include "ethernet.h"


int ethernet_init(struct ethernet_device_info_t *dev_info) {
    if (create_client(dev_info) < 0) {
        perror("Ethernet:Error create client");
    }
}

int ethernet_deinit(struct ethernet_device_info_t *dev_info) {
    close(dev_info->client_fd);
}

int create_client(struct ethernet_device_info_t *dev_info) {

   int err;
   if ((dev_info->client_fd = socket(AF_INET , SOCK_STREAM , 0)) < 0) {
        perror("Error while opening socket");
        return -1;
    }
    dev_info->addr.sin_family = PF_INET;
    dev_info->addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    dev_info->addr.sin_port = htons(8700);

    err = connect(dev_info->client_fd,(struct sockaddr *)&dev_info->addr,sizeof(dev_info->addr));
    if(err == -1){
        printf("ethernet client: Connection error !!\n");
    }

}

int create_server(struct ethernet_device_info_t *dev_info) {

}

uint32_t ethernet_data_recv(int sockfd, uint8_t *rx_buff, uint32_t buff_size) {
    
    uint32_t offset = 0;
    int32_t rdlen = 0;
    do {
            rdlen = recv(sockfd, rx_buff + offset, buff_size - offset, 0);
            offset += rdlen;
    } while (offset < buff_size && rdlen > 0);
    return offset;
}

uint32_t ethernet_data_send(int sockfd, uint8_t *tx_buff, uint32_t buff_size) {
    
    uint32_t offset = 0;
    int32_t wdlen = 0;
    
    while (offset < buff_size) {
            wdlen = send(sockfd, tx_buff + offset, buff_size - offset, 0);
            offset += wdlen;
    }
    return offset;
}