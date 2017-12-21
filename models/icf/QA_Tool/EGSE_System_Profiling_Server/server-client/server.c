#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "icf_utility.h"
#define BUFSIZE 1024

#if 1


#define SERVER_PORT  8700

#define TRUE             1
#define FALSE            0

int main (int argc, char *argv[]) {
    int    i, len, rc, on = 1;
    int    listen_sd, max_sd, new_sd;
    int    desc_ready, end_server = FALSE;
    int    close_conn;
    char   buffer[80];
    struct sockaddr_in   addr;
    struct timeval       timeout;
    fd_set master_set, working_set;

    /*************************************************************/
    /* Create an AF_INET stream socket to receive incoming       */
    /* connections on                                            */
    /*************************************************************/
    listen_sd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sd < 0) {
        perror("socket() failed");
        exit(-1);
    }

    /*************************************************************/
    /* Allow socket descriptor to be reuseable                   */
    /*************************************************************/
    rc = setsockopt(listen_sd, SOL_SOCKET,  SO_REUSEADDR,
                    (char *)&on, sizeof(on));
    if (rc < 0) {
        perror("setsockopt() failed");
        close(listen_sd);
        exit(-1);
    }

    /*************************************************************/
    /* Set socket to be nonblocking. All of the sockets for    */
    /* the incoming connections will also be nonblocking since  */
    /* they will inherit that state from the listening socket.   */
    /*************************************************************/
    rc = ioctl(listen_sd, FIONBIO, (char *)&on);
    if (rc < 0) {
        perror("ioctl() failed");
        close(listen_sd);
        exit(-1);
    }

    /*************************************************************/
    /* Bind the socket                                           */
    /*************************************************************/
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(SERVER_PORT);
    rc = bind(listen_sd,
              (struct sockaddr *)&addr, sizeof(addr));
    if (rc < 0) {
        perror("bind() failed");
        close(listen_sd);
        exit(-1);
    }

    /*************************************************************/
    /* Set the listen back log                                   */
    /*************************************************************/
    rc = listen(listen_sd, 32);
    if (rc < 0) {
        perror("listen() failed");
        close(listen_sd);
        exit(-1);
    }

    /*************************************************************/
    /* Initialize the master fd_set                              */
    /*************************************************************/
    FD_ZERO(&master_set);
    max_sd = listen_sd;
    FD_SET(listen_sd, &master_set);

    /*************************************************************/
    /* Initialize the timeval struct to 3 minutes.  If no        */
    /* activity after 3 minutes this program will end.           */
    /*************************************************************/
    timeout.tv_sec  = 3 * 60;
    timeout.tv_usec = 0;

    /*************************************************************/
    /* Loop waiting for incoming connects or for incoming data   */
    /* on any of the connected sockets.                          */
    /*************************************************************/
    do {
        /**********************************************************/
        /* Copy the master fd_set over to the working fd_set.     */
        /**********************************************************/
        memcpy(&working_set, &master_set, sizeof(master_set));

        /**********************************************************/
        /* Call select() and wait 5 minutes for it to complete.   */
        /**********************************************************/
        printf("Waiting on select()...\n");
        rc = select(max_sd + 1, &working_set, NULL, NULL, &timeout);

        /**********************************************************/
        /* Check to see if the select call failed.                */
        /**********************************************************/
        if (rc < 0) {
            perror("  select() failed");
            break;
        }

        /**********************************************************/
        /* Check to see if the 5 minute time out expired.         */
        /**********************************************************/
        if (rc == 0) {
            printf("  select() timed out.  End program.\n");
            break;
        }

        /**********************************************************/
        /* One or more descriptors are readable.  Need to         */
        /* determine which ones they are.                         */
        /**********************************************************/
        desc_ready = rc;
        for (i = 0; i <= max_sd  &&  desc_ready > 0; ++i) {
            /*******************************************************/
            /* Check to see if this descriptor is ready            */
            /*******************************************************/
            if (FD_ISSET(i, &working_set)) {
                /****************************************************/
                /* A descriptor was found that was readable - one   */
                /* less has to be looked for.  This is being done   */
                /* so that we can stop looking at the working set   */
                /* once we have found all of the descriptors that   */
                /* were ready.                                      */
                /****************************************************/
                desc_ready -= 1;

                /****************************************************/
                /* Check to see if this is the listening socket     */
                /****************************************************/
                if (i == listen_sd) {
                    printf("  Listening socket is readable\n");
                    /*************************************************/
                    /* Accept all incoming connections that are      */
                    /* queued up on the listening socket before we   */
                    /* loop back and call select again.              */
                    /*************************************************/
                    do {
                        /**********************************************/
                        /* Accept each incoming connection.  If       */
                        /* accept fails with EWOULDBLOCK, then we     */
                        /* have accepted all of them.  Any other      */
                        /* failure on accept will cause us to end the */
                        /* server.                                    */
                        /**********************************************/
                        new_sd = accept(listen_sd, NULL, NULL);
                        if (new_sd < 0) {
                            if (errno != EWOULDBLOCK) {
                                perror("  accept() failed");
                                end_server = TRUE;
                            }
                            break;
                        }

                        /**********************************************/
                        /* Add the new incoming connection to the     */
                        /* master read set                            */
                        /**********************************************/
                        printf("  New incoming connection - %d\n", new_sd);
                        FD_SET(new_sd, &master_set);
                        if (new_sd > max_sd)
                            max_sd = new_sd;

                        /**********************************************/
                        /* Loop back up and accept another incoming   */
                        /* connection                                 */
                        /**********************************************/
                    } while (new_sd != -1);
                }

                /****************************************************/
                /* This is not the listening socket, therefore an   */
                /* existing connection must be readable             */
                /****************************************************/
                else {
                    printf("  Descriptor %d is readable\n", i);
                    close_conn = FALSE;
                    /*************************************************/
                    /* Receive all incoming data on this socket      */
                    /* before we loop back and call select again.    */
                    /*************************************************/
                    do {
                        /**********************************************/
                        /* Receive data on this connection until the  */
                        /* recv fails with EWOULDBLOCK.  If any other */
                        /* failure occurs, we will close the          */
                        /* connection.                                */
                        /**********************************************/
                        rc = recv(i, buffer, sizeof(buffer), 0);
                        if (rc < 0) {
                            if (errno != EWOULDBLOCK) {
                                perror("  recv() failed");
                                close_conn = TRUE;
                            }
                            break;
                        }

                        /**********************************************/
                        /* Check to see if the connection has been    */
                        /* closed by the client                       */
                        /**********************************************/
                        if (rc == 0) {
                            printf("  Connection closed\n");
                            close_conn = TRUE;
                            break;
                        }

                        /**********************************************/
                        /* Data was received                          */
                        /**********************************************/
                        len = rc;
                        hex_dump("RX Eth", buffer, len);

                        /**********************************************/
                        /* Echo the data back to the client           */
                        /**********************************************/
#if 0
                        rc = send(i, buffer, len, 0);
                        if (rc < 0) {
                            perror("  send() failed");
                            close_conn = TRUE;
                            break;
                        }
#endif
                    } while (TRUE);

                    /*************************************************/
                    /* If the close_conn flag was turned on, we need */
                    /* to clean up this active connection.  This     */
                    /* clean up process includes removing the        */
                    /* descriptor from the master set and            */
                    /* determining the new maximum descriptor value  */
                    /* based on the bits that are still turned on in */
                    /* the master set.                               */
                    /*************************************************/
                    if (close_conn) {
                        close(i);
                        FD_CLR(i, &master_set);
                        if (i == max_sd) {
                            while (FD_ISSET(max_sd, &master_set) == FALSE)
                                max_sd -= 1;
                        }
                    }
                } /* End of existing connection is readable */
            } /* End of if (FD_ISSET(i, &working_set)) */
        } /* End of loop through selectable descriptors */

    } while (end_server == FALSE);

    /*************************************************************/
    /* Clean up all of the sockets that are open                  */
    /*************************************************************/
    for (i = 0; i <= max_sd; ++i) {
        if (FD_ISSET(i, &master_set))
            close(i);
    }
}
#endif

#if 0
int main(int argc , char *argv[])

{
    uint8_t inputBuffer[24] = {};
    struct timeval tv;
    int sockfd = 0, forClientSockfd = 0;
    struct sockaddr_in serverInfo, clientInfo;
    int addrlen = sizeof(clientInfo);

    fd_set readfds;
    tv.tv_sec = 0;
    tv.tv_usec = 100;

    sockfd = socket(AF_INET , SOCK_STREAM , 0);


    bzero(&serverInfo, sizeof(serverInfo));
    serverInfo.sin_family = PF_INET;
    serverInfo.sin_addr.s_addr = INADDR_ANY;
    serverInfo.sin_port = htons(8700);

    if (sockfd == -1) {
        printf("Fail to create a socket.");
    }
    if (bind(sockfd, (struct sockaddr *)&serverInfo, sizeof(serverInfo)) < 0) {
        perror ("bind failed");
        exit(1);
    }

    if (listen(sockfd, 5) < 0) {
        perror ("listen failed");
        exit(1);
    }

    FD_ZERO(&readfds);


    while (1) {
        forClientSockfd = accept(sockfd, (struct sockaddr*) &clientInfo, &addrlen);
        FD_SET(sockfd, &readfds);
        select(sockfd + 1, &readfds, NULL, NULL, &tv);
        if (FD_ISSET(sockfd, &readfds)) {
            recv(forClientSockfd, inputBuffer, sizeof(inputBuffer), 0);
            hex_dump("Server", inputBuffer, sizeof(inputBuffer));
        }
    }
    return 0;
}
#endif
#if 0
void error(char *msg) {
    perror(msg);
    exit(1);
}

int main(int argc, char **argv) {
    int parentfd; /* parent socket */
    int client_fd; /* child socket */
    int portno; /* port to listen on */
    int clientlen; /* byte size of client's address */
    struct sockaddr_in serveraddr; /* server's addr */
    struct sockaddr_in clientaddr; /* client addr */
    char buf[BUFSIZE]; /* message buffer */
    int optval; /* flag value for setsockopt */
    int nbytes; /* message byte size */
    int connectcnt; /* number of connection requests */
    int notdone;
    fd_set readfds;

    /*
     * check command line arguments
     */
    if (argc != 2) {
        fprintf(stderr, "usage: %s <port>\n", argv[0]);
        exit(1);
    }
    portno = atoi(argv[1]);

    /*
     * socket: create the parent socket
     */
    parentfd = socket(AF_INET, SOCK_STREAM, 0);
    if (parentfd < 0)
        error("ERROR opening socket");

    /* setsockopt: Handy debugging trick that lets
     * us rerun the server immediately after we kill it;
     * otherwise we have to wait about 20 secs.
     * Eliminates "ERROR on binding: Address already in use" error.
     */
    optval = 1;
    setsockopt(parentfd, SOL_SOCKET, SO_REUSEADDR,
               (const void *)&optval , sizeof(int));

    /*
     * build the server's Internet address
     */
    bzero((char *) &serveraddr, sizeof(serveraddr));

    /* this is an Internet address */
    serveraddr.sin_family = AF_INET;

    /* let the system figure out our IP address */
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);

    /* this is the port we will listen on */
    serveraddr.sin_port = htons((unsigned short)portno);

    /*
     * bind: associate the parent socket with a port
     */
    if (bind(parentfd, (struct sockaddr *) &serveraddr,
             sizeof(serveraddr)) < 0)
        error("ERROR on binding");

    /*
     * listen: make this socket ready to accept connection requests
     */
    if (listen(parentfd, 1) < 0) /* allow 1 requests to queue up */
        error("ERROR on listen");


    /* initialize some things for the main loop */
    clientlen = sizeof(clientaddr);
    notdone = 1;
    connectcnt = 0;
    printf("server> ");
    fflush(stdout);

    /*
     * main loop: wait for connection request or stdin command.
     *
     * If connection request, then echo input line
     * and close connection.
     * If command, then process command.
     */
    FD_ZERO(&readfds);          /* initialize the fd set */

    while (notdone) {

        /*
         * select: Has the user typed something to stdin or
         * has a connection request arrived?
         */
        FD_SET(parentfd, &readfds); /* add socket fd */
        FD_SET(0, &readfds);        /* add stdin fd (0) */
        if (select(parentfd + 1, &readfds, 0, 0, 0) < 0) {
            error("ERROR in select");
        }

        /* if the user has entered a command, process it */
        if (FD_ISSET(0, &readfds)) {
            fgets(buf, BUFSIZE, stdin);
            switch (buf[0]) {
            case 'c': /* print the connection cnt */
                printf("Received %d connection requests so far.\n", connectcnt);
                printf("server> ");
                fflush(stdout);
                break;
            case 'q': /* terminate the server */
                notdone = 0;
                break;
            default: /* bad input */
                printf("ERROR: unknown command\n");
                printf("server> ");
                fflush(stdout);
            }
        }

        /* if a connection request has arrived, process it */
        if (FD_ISSET(parentfd, &readfds)) {
            /*
             * accept: wait for a connection request
             */

            client_fd = accept(parentfd, (struct sockaddr *) &clientaddr, &clientlen);
            if (client_fd < 0)
                error("ERROR on accept");
            connectcnt++;
            /*
             * read: read input string from the client
             */
            bzero(buf, BUFSIZE);
            //nbytes = read(client_fd, buf, BUFSIZE);
            nbytes = recv(client_fd, buf, BUFSIZE, 0);
            if (nbytes < 0)
                error("ERROR reading from socket");
            hex_dump("RX buff", buf , nbytes);
            /*
             * write: echo the input string back to the client
             */
            // n = write(client_fd, buf, strlen(buf));
            // if (n < 0)
            //     error("ERROR writing to socket");
            if (!notdone) {
                close(client_fd);
                client_fd = 0;
            }
        }
    }

    /* clean up */
    printf("Terminating server.\n");
    close(parentfd);
    exit(0);
}
#endif