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

    char head, type, unit, digits, high, low;
    unsigned long long time_stamp = 0;
    double ppm;

    for (int i = 0; ; i = 0, time_stamp++) {
        do {
            head = serialGetchar(serial_fd);
            if (head == -1) {
                perror("serialGetchar");
                goto error;
            }
        } while (head != 0xff);
        
        type = serialGetchar(serial_fd);
        if (type == -1) {
            perror("serialGetchar");
            goto error;
        }

        unit = serialGetchar(serial_fd);
        if (unit == -1) {
            perror("serialGetchar");
            goto error;
        }

        digits = serialGetchar(serial_fd);
        if (digits == -1) {
            perror("serialGetchar");
            goto error;
        }

        high = serialGetchar(serial_fd);
        if (high == -1) {
            perror("serialGetchar");
            goto error;
        }

        low  = serialGetchar(serial_fd);
        if (low == -1) {
            perror("serialGetchar");
            goto error;
        }

        ppm = (high * 256 + low) / 1000.0;

        int len = snprintf(buffer, MAX_BUFFER_SIZE,
                           "{\n\t\"timeStamp\": %llu,\n\t\"type\": \"olfactory\",\n\t\"ppm\": %.3f\n}\r\n", time_stamp, ppm);
        if (len > MAX_BUFFER_SIZE) {
            fprintf(stderr, "Payload is incomplete. Please increase SOCKET_BUFFER_SIZE.\n");
            goto error;
        }
        if (send(socket_fd, buffer, len, 0) < 0) {
            printf("connection closed\n");
            goto success;
        }
        printf("%s", buffer);
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
