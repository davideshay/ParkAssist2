#include <iostream>
#include <cstring>
#include <string>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdlib>

int main(int argc, char* argv[]) {
    // Default port is 44444, but can be overridden by command line argument
    unsigned short port = 44444;
    
    // Check if a port number was provided as command line argument
    if (argc > 1) {
        int provided_port = std::atoi(argv[1]);
        if (provided_port > 0 && provided_port <= 65535) {
            port = static_cast<unsigned short>(provided_port);
        } else {
            std::cerr << "Invalid port number. Using default port 44444." << std::endl;
        }
    }

    // Create a UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Failed to create socket: " << strerror(errno) << std::endl;
        return 1;
    }

    // Enable SO_REUSEADDR to allow binding to a port that might be in TIME_WAIT state
    int reuse = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "Failed to set SO_REUSEADDR: " << strerror(errno) << std::endl;
        close(sock);
        return 1;
    }

    // Enable SO_REUSEPORT for macOS
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "Failed to set SO_REUSEPORT: " << strerror(errno) << std::endl;
        close(sock);
        return 1;
    }

    // For receiving broadcast packets
    int broadcast = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
        std::cerr << "Failed to set SO_BROADCAST: " << strerror(errno) << std::endl;
        close(sock);
        return 1;
    }

    // Bind socket to the specified port
    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;  // Bind to all interfaces

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Failed to bind to port " << port << ": " << strerror(errno) << std::endl;
        close(sock);
        return 1;
    }

    std::cout << "Listening for UDP packets on port " << port << "..." << std::endl;

    // Receive loop
    while (true) {
        char buffer[4096];
        struct sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);

        ssize_t received = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, 
                                   (struct sockaddr*)&sender_addr, &sender_len);

        if (received < 0) {
            std::cerr << "Error receiving packet: " << strerror(errno) << std::endl;
            continue;
        }

        // Null-terminate the buffer
        buffer[received] = '\0';

        // Print the ASCII contents followed by a carriage return
        std::cout << buffer << std::endl;
    }

    close(sock);
    return 0;
}
