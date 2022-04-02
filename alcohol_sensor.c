#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <wiringSerial.h>

#define MAX_BUFFER_SIZE 1024

char buffer[MAX_BUFFER_SIZE];
int socket_fd = -1, serial_fd = -1;

void signal_handler(int signal) {
    if (socket_fd >= 0) {
        close(socket_fd);
    }
    if (serial_fd >= 0) {
        serialClose(serial_fd);
    }
    exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[]) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (argc != 4) {
        fprintf(stderr, "usage: %s <IP_ADDRESS> <PORT> <DEVICE>\n", argv[0]);
        return EXIT_FAILURE;
    }
    
    struct sockaddr_in socket_address;
    socket_address.sin_family = AF_INET;
    socket_address.sin_addr.s_addr = inet_addr(argv[1]);
    socket_address.sin_port = htons(atoi(argv[2]));
    socklen_t socket_address_size = sizeof(socket_address);

    if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("socket");
        return EXIT_FAILURE;
    }

    if (connect(socket_fd, (struct sockaddr *) &socket_address, socket_address_size) < 0) {
        perror("connect");
        goto error;
    }
    
    if ((serial_fd = serialOpen(argv[3], 9600)) < 0) {
        perror("serialOpen");
        goto error;
    }

    char head, buf[14];
    unsigned long long time_stamp = 0;
    double voltage;
    
    serialPuts(serial_fd, "AT+V\r\n");
    do {
        head = serialGetchar(serial_fd);
        if (head == -1) {
            perror("serialGetchar");
            goto error;
        }
    } while (head != '.');
    do {
        head = serialGetchar(serial_fd);
        if (head == -1) {
            perror("serialGetchar");
            goto error;
        }
    } while (head != '\n');
    
    for (int i = 0; ; i = 0, time_stamp++) {
        serialPuts(serial_fd, "AT+V\r\n");
        for (int j = 0; j < 13; j++) {
            buf[i] = serialGetchar(serial_fd);
            if (buf[i++] == -1) {
                perror("serialGetchar");
                goto error;
            }
        }
        buf[i] = '\0';
        sscanf(buf, "OK\r\n+V=%lf", &voltage);
        // printf("%s", buf);
        // printf("%.2f\n", voltage);
        int len = snprintf(buffer, MAX_BUFFER_SIZE,
                           "{\n\t\"timeStamp\": %llu,\n\t\"type\": \"olfactory\",\n\t\"voltage\": %.2f\n}\n\r", time_stamp, voltage);
        if (len > MAX_BUFFER_SIZE) {
            fprintf(stderr, "Payload is incomplete. Please increase SOCKET_BUFFER_SIZE.\n");
            goto error;
        }
        if (send(socket_fd, buffer, len, 0) < 0) {
            printf("connection closed\n");
            goto success;
        }
    }
success:
    if (socket_fd >= 0) {
        close(socket_fd);
    }
    if (serial_fd >= 0) {
        serialClose(serial_fd);
    }
    return EXIT_SUCCESS;
error:
    if (socket_fd >= 0) {
        close(socket_fd);
    }
    if (serial_fd >= 0) {
        serialClose(serial_fd);
    }
    return EXIT_FAILURE;
}
