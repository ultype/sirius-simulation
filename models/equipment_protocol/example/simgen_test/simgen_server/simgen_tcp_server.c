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

int tcp_data_recv(int client_fd, uint8_t *rx_buff, uint32_t buff_size) {
    uint32_t offset = 0;
    int32_t rdlen = 0;

    while (offset < buff_size) {
        if ((rdlen = recv(client_fd, rx_buff + offset, buff_size - offset, 0)) < 0) {
            if (errno == EINTR)
                rdlen = 0;
            else
                return -1;
        } else if (rdlen == 0) {
            break;
        }
        offset += rdlen;
    }
    return offset;
}


int main(int argc, char const *argv[]) {
    int tcp_serv_fd;   /* file description into transport */
    int tcp_client_fd;   /* file description into transport */
    int nbytes;
    int length; /* length of address structure      */
    char buf[MAXNAME];
    int optval = 1; /* prevent from address being taken */
    struct sockaddr_in tcpaddr; /* address of this service */
    struct sockaddr_in tcp_client_addr; /* address of client    */
    char message[] = "TCP ACK";
    struct timeval timeout={1,0};//1s
    /*
     *      Get a socket into UDP/IP
     */
    if ((tcp_serv_fd = socket(AF_INET, SOCK_STREAM, 0)) <0) {
        perror ("socket failed");
        goto EXIT_SERVER;
    }
    if (setsockopt(tcp_serv_fd, SOL_SOCKET,  SO_REUSEADDR, (char *)&optval, sizeof(int)) < 0) {
        goto EXIT_SERVER;
    }
    //  setsockopt(tcp_serv_fd, SOL_SOCKET,SO_SNDTIMEO, &timeout, sizeof(timeout));
    /*
     *    Set up our address
     */
    bzero ((char *)&tcpaddr, sizeof(tcpaddr));
    tcpaddr.sin_family = AF_INET;
    tcpaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    tcpaddr.sin_port = htons(SERV_PORT);

    /*
     *     Bind to the address to which the service will be offered
     */
    if (bind(tcp_serv_fd, (struct sockaddr *)&tcpaddr, sizeof(tcpaddr)) <0) {
        perror ("bind failed\n");
        goto EXIT_SERVER;
    }
    if (listen(tcp_serv_fd, 5) < 0) {
        fprintf(stderr, "listen() failed\n");
        goto EXIT_SERVER;
    }

    /*
     * Loop continuously, waiting for datagrams
     * and response a message
     */
    length = sizeof(tcp_client_addr);
    tcp_client_fd = accept(tcp_serv_fd, (struct sockaddr*)&(tcp_client_addr), (socklen_t *)&length);
    if (tcp_client_fd < 0) {
        fprintf(stderr, "create_server: Accept Fail ... :%d\n", SERV_PORT);
        goto EXIT_SERVER;
    }
    printf("TCP Server is ready to receive !!\n");
    printf("Can strike Cntrl-c to stop Server >>\n");
    while (1) {
        memset(buf, 0, MAXNAME);
        if ((nbytes = recv(tcp_client_fd, (uint8_t *)&buf, MAXNAME, 0)) > 0) {
            printf("Received data form %s : %d\n", inet_ntoa(tcp_client_addr.sin_addr), htons(tcp_client_addr.sin_port));
            printf("%s", (char *)&buf);
            send(tcp_client_fd, message, sizeof(message), 0);
            printf("Send resp ...\n\n");
        }
    }
EXIT_SERVER:
    close(tcp_serv_fd);
    return 0;
}