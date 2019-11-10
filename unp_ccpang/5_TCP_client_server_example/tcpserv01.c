#include "unp.h"

int main(int argc, char **argv) {

    int listenfd, connfd;
    pid_t  childpid;
    socklen_t chilen;
    struct sockaddr_in cliaddr, servaddr;

    listenfd = Socket (AF_INET, SOCK_STREAM, 0);

    bzeo(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(SERV_PORT);

    Bind()
}
