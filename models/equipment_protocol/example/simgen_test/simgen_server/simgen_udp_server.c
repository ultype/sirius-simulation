#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/errno.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h> // for close
#define SERV_PORT 15650

#define MAXNAME 1024

static int udp_cmd_recvfrom(int client_fd , uint8_t *payload, uint32_t frame_len, struct sockaddr *src_addr, socklen_t *addrlen) {
    uint32_t offset = 0;
    int32_t wdlen = 0;
    while (offset < frame_len) {
        if ((wdlen = recvfrom(client_fd, payload + offset, frame_len - offset, 0, src_addr, addrlen)) < 0) {
            if (wdlen < 0 && errno == EINTR)
                wdlen = 0;
            else
                return -1;
        }
        offset += wdlen;
    }
    return offset;
}
int main(int argc, char const *argv[]) {
    int udp_serv_fd;   /* file description into transport */
    int length; /* length of address structure      */
    int nbytes; /* the number of read **/
    char buf[BUFSIZ];
    struct sockaddr_in udpaddr; /* address of this service */
    struct sockaddr_in udp_client_addr; /* address of client    */
    /*
     *      Get a socket into UDP/IP
     */
    if ((udp_serv_fd = socket(AF_INET, SOCK_DGRAM, 0)) <0) {
        perror ("socket failed");
        exit(EXIT_FAILURE);
    }
    /*
     *    Set up our address
     */
    bzero ((char *)&udpaddr, sizeof(udpaddr));
    udpaddr.sin_family = AF_INET;
    udpaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    udpaddr.sin_port = htons(SERV_PORT);

    /*
     *     Bind to the address to which the service will be offered
     */
    if (bind(udp_serv_fd, (struct sockaddr *)&udpaddr, sizeof(udpaddr)) <0) {
        perror ("bind failed\n");
        exit(1);
    }
    /*
     * Loop continuously, waiting for datagrams
     * and response a message
     */
    length = sizeof(udp_client_addr);
    printf("UDP Server is ready to receive !!\n");
    printf("Can strike Cntrl-c to stop Server >>\n");
    while (1) {
        if ((nbytes = udp_cmd_recvfrom(udp_serv_fd, (uint8_t *)&buf, MAXNAME, (struct sockaddr*)&udp_client_addr, (socklen_t *)&length)) < 0) {
            perror ("could not read datagram!!");
            continue;
        }


        printf("Received data form %s : %d\n", inet_ntoa(udp_client_addr.sin_addr), htons(udp_client_addr.sin_port));
        printf("%s\n", buf);

        printf("Can Strike Crtl-c to stop Server >>\n");
    }
    close(udp_serv_fd);
    return 0;
}