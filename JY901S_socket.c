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

    unsigned char head, type, buf[10];
    short *recv_data = (short *) buf;
    unsigned long long time_stamp = 0;
    float temperature, acceleration[3], angular_speed[3], azimuth[3], magnetic_intensity[3];
    int len;
    
    while (1) {
        do {
            head = serialGetchar(serial_fd);
            if (head == -1) {
                perror("serialGetchar");
                goto error;
            }
        } while (head != 0x55);
        
        type = serialGetchar(serial_fd);
        if (type == -1) {
            perror("serialGetchar");
            goto error;
        }
        
        for (int i = 0; i < 9; i++) {
            buf[i] = serialGetchar(serial_fd);
            if (buf[i] == -1) {
                perror("serialGetchar");
                goto error;
            }
        }
        
        switch (type) {
        case 0x51:
            for (int i = 0; i < 3; i++) {
                acceleration[i] = (float) recv_data[i] / 32768.0 * 16.0 * 9.80665;
            }
            temperature = recv_data[3] / 100.0;
            len = snprintf(buffer, MAX_BUFFER_SIZE,
                           "{\n\t\"timeStamp\": %llu,\n\t\"type\": \"motion\", \n\t\"temperature\": %2.2f,\n\t\"acceleration\": {\"x\": %6.3f, \"y\": %6.3f, \"z\": %6.3f}",
                           time_stamp++, temperature, acceleration[0], acceleration[1], acceleration[2]);
            if (len > MAX_BUFFER_SIZE) {
                fprintf(stderr, "Payload is incomplete. Please increase SOCKET_BUFFER_SIZE.\n");
                goto error;
            }
            break;
        case 0x52:
            for (int i = 0; i < 3; i++) {
                angular_speed[i] = (float) recv_data[i] / 32768.0 * 2000.0;
            }
            sprintf(buffer, "%s,\n\t\"angularSpeed\": {\"x\": %7.3f, \"y\": %7.3f, \"z\": %7.3f}", buffer, angular_speed[0], angular_speed[1], angular_speed[2]);
            break;
        case 0x53:
            for (int i = 0; i < 3; i++) {
                azimuth[i] = (float) recv_data[i] / 32768.0 * 180.0;
            }
            sprintf(buffer, "%s,\n\t\"azimuth\": {\"row\": %7.3f, \"pitch\": %7.3f, \"yaw\": %7.3f}", buffer, azimuth[0], azimuth[1], azimuth[2]);
            break;
        case 0x54:
            for (int i = 0; i < 3; i++) {
                magnetic_intensity[i] = (float) recv_data[i];
            }
            len = sprintf(buffer, "%s,\n\t\"magneticIntensity\": {\"x\": %4.0f, \"y\": %4.0f, \"z\": %4.0f}\n}\r\n", buffer, magnetic_intensity[0], magnetic_intensity[1], magnetic_intensity[2]);
            if (send(socket_fd, buffer, len, 0) < 0) {
                printf("connection closed\n");
                goto success;
            }
            break;
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
