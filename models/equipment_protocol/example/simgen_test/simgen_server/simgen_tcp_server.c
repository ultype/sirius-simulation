#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/errno.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h> // for close
#include <fcntl.h>
#include <sys/ioctl.h>
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
    int length; /* length of address structure      */
    char buf[MAXNAME];
    int optval = 1; /* prevent from address being taken */
    struct sockaddr_in tcpaddr; /* address of this service */
    struct sockaddr_in tcp_client_addr; /* address of client    */
    char message[] = "TCP ACK";
    struct timeval timeout={60*5,0};//1s
    int err = 0;
    fd_set master_set, working_set;
    int    max_sd, i = 0;
    int    desc_ready, end_server = 0;
    int    close_conn;
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
    optval = fcntl(tcp_serv_fd, F_GETFL);
    optval = optval | O_NONBLOCK;
    err = fcntl(tcp_serv_fd, F_SETFL, optval);
    //  err = ioctl(listen_sd, FIONBIO, (char *)&optval);
    if (err < 0) {
        perror("ioctl() failed");
        close(tcp_serv_fd);
        exit(-1);
    }

    if (listen(tcp_serv_fd, 5) < 0) {
        fprintf(stderr, "listen() failed\n");
        goto EXIT_SERVER;
    }

    FD_ZERO(&master_set);
    max_sd = tcp_serv_fd;
    FD_SET(tcp_serv_fd, &master_set);

    length = sizeof(tcp_client_addr);
    printf("TCP Server is ready to receive !!\n");
    printf("Can strike Cntrl-c to stop Server >>\n");

    do {
        memcpy(&working_set, &master_set, sizeof(master_set));
        printf("Waiting on select()...\n");
        err = select(max_sd + 1, &working_set, NULL, NULL, &timeout);

        if (err < 0) {
            perror("  select() failed");
            break;
        }

        if (err == 0) {
            printf("  select() timed out.  End program.\n");
            break;
        }
        desc_ready = err;
        for (i = 0; i <= max_sd  &&  desc_ready > 0; ++i) {
            if (FD_ISSET(i, &working_set)) {
                desc_ready -= 1;
                if (i == tcp_serv_fd) {
                    printf("  Listening socket is readable\n");
                    do {
                        tcp_client_fd = accept(tcp_serv_fd, (struct sockaddr*)&(tcp_client_addr), (socklen_t *)&length);
                        if (tcp_client_fd < 0) {
                            if (errno != EWOULDBLOCK) {
                                perror("  accept() failed");
                                end_server = 1;
                            }
                            break;
                        }

                        printf("  New incoming connection - %d\n", tcp_client_fd);
                        FD_SET(tcp_client_fd, &master_set);
                        if (tcp_client_fd > max_sd)
                            max_sd = tcp_client_fd;
                    } while (tcp_client_fd != -1);
                } else {
                    printf("  Descriptor %d is readable\n", i);
                    close_conn = 0;

                    do {
                        memset(buf, 0, MAXNAME);
                        err = recv(i, (uint8_t *)&buf, MAXNAME, 0);
                        if (err < 0) {
                            if (errno != EWOULDBLOCK) {
                                perror("  recv() failed");
                                close_conn = 1;
                            }
                            break;
                        }
                        if (err == 0) {
                            printf("  Connection closed\n");
                            close_conn = 1;
                            break;
                        }
                        printf("Received data form %s : %d\n", inet_ntoa(tcp_client_addr.sin_addr), htons(tcp_client_addr.sin_port));
                        printf("%s", (char *)&buf);
                        err = send(i, message, sizeof(message), 0);
                        printf("Send resp ...\n\n");
                        if (err < 0) {
                            perror("  send() failed");
                            close_conn = 1;
                            break;
                        }

                    } while (1);
                    if (close_conn) {
                        close(i);
                        FD_CLR(i, &master_set);
                        if (i == max_sd) {
                            while (FD_ISSET(max_sd, &master_set) == 0)
                                max_sd -= 1;
                        }
                    }
                } /* End of existing connection is readable */
            } /* End of if (FD_ISSET(i, &working_set)) */
        } /* End of loop through selectable descriptors */

    } while (end_server == 0);

    for (i = 0; i <= max_sd; ++i) {
        if (FD_ISSET(i, &master_set))
            close(i);
    }
EXIT_SERVER:
    close(tcp_serv_fd);
    return 0;
}