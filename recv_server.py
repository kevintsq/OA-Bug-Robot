IPC_TYPE = "SOCKET"
if IPC_TYPE == "SOCKET":
    import socket

    HOST = ''
    PORT = 8080
    TRUNK_SIZE = 1024
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR | socket.SO_REUSEPORT, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        connection, address = s.accept()
        with connection:
            print(f'Connected by {address}.')
            while True:
                data = connection.recv(TRUNK_SIZE).decode('ascii')
                if not data:
                    print(f"Connection closed from {address}.")
                    break
                print(data)
elif IPC_TYPE == "PIPE":
    import subprocess

    with subprocess.Popen("./client", stdout=subprocess.PIPE, text=True) as p:
        while True:
            data = p.stdout.readline()
            if not data:
                print("Connection closed.")
                break
            print(data, end='')
