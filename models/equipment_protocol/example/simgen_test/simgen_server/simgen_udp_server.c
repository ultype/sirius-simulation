#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/errno.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h> // for close
#include <time.h>
#define SERV_PORT 15650

#define MAXNAME 1024
void hex_dump(char *str, uint8_t *pSrcBufVA, uint32_t SrcBufLen) {
    uint8_t *pt;
    uint32_t x;
    pt = pSrcBufVA;
    printf("%s: %p, len = %d\n\r", str, pSrcBufVA, SrcBufLen);
    for (x = 0; x < SrcBufLen; x++) {
        if (x % 16 == 0) {
            printf("0x%04x : ", x);
        }
        printf("%02x ", ((uint8_t)pt[x]));
        if (x % 16 == 15) { printf("\n\r"); }
    }
    printf("\n\r");
}

int main(int argc, char const *argv[]) {
    int udp_serv_fd;   /* file description into transport */
    int length; /* length of address structure      */
    int nbytes; /* the number of read **/
    char buf[MAXNAME];
    struct sockaddr_in udpaddr; /* address of this service */
    struct sockaddr_in udp_client_addr; /* address of client    */
    char date_buf[80];
    char currentTime[84] = "";
    static struct timespec ts;
    uint32_t milli;
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
        if ((nbytes = (recvfrom(udp_serv_fd, buf, MAXNAME, 0, (struct sockaddr*)&udp_client_addr, (socklen_t *)&length))) > 0) {
            clock_gettime(CLOCK_MONOTONIC, &ts);
            ts.tv_sec = time(NULL);
            milli = ts.tv_nsec / 1000000;
            strftime(date_buf, (size_t) 20, "%Y/%m/%d,%H:%M:%S", localtime(&ts.tv_sec));
            snprintf(currentTime, sizeof(currentTime), "%s.%03d", date_buf, milli);
            printf("[%s]Received data form %s : %d\n", currentTime, inet_ntoa(udp_client_addr.sin_addr), htons(udp_client_addr.sin_port));
            //  hex_dump("UDP DUMP", (uint8_t *)&buf, nbytes);
        }

    }
    close(udp_serv_fd);
    return 0;
}