#include <iostream>
#include <cstring>
#include <string>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdlib>
#include <ifaddrs.h>
#include <net/if.h>
#include <fcntl.h>
#include <errno.h>

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

    // Another important socket option for some systems
    int yes = 1;
    // For macOS, ensure we can reuse the address
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes)) < 0) {
        std::cerr << "Warning: Failed to set SO_REUSEPORT again: " << strerror(errno) << std::endl;
    }

    // Allow binding to addresses that are not local to the machine
    // This can help with receiving broadcasts on macOS
    #ifdef SO_BINDANY
    if (setsockopt(sock, SOL_SOCKET, SO_BINDANY, &yes, sizeof(yes)) < 0) {
        std::cerr << "Warning: Failed to set SO_BINDANY: " << strerror(errno) << std::endl;
    }
    #endif

    // Make the socket non-blocking so we can detect errors immediately
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    // Bind socket to the specified port on INADDR_ANY (0.0.0.0)
    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);  // Bind to all interfaces: 0.0.0.0

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Failed to bind to port " << port << ": " << strerror(errno) << std::endl;
        close(sock);
        return 1;
    }

    // On some systems, we might need to join multicast groups to receive all broadcast traffic
    struct ifaddrs *interfaces = NULL;
    if (getifaddrs(&interfaces) == 0) {
        std::cout << "Network interfaces:" << std::endl;
        for (struct ifaddrs *ifa = interfaces; ifa != NULL; ifa = ifa->ifa_next) {
            if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET) {
                // Skip loopback interface
                if (!(ifa->ifa_flags & IFF_LOOPBACK)) {
                    struct sockaddr_in *addr = (struct sockaddr_in *)ifa->ifa_addr;
                    std::cout << "  " << ifa->ifa_name << ": " 
                              << inet_ntoa(addr->sin_addr) << std::endl;
                    
                    // For interfaces that can receive broadcast
                    if (ifa->ifa_flags & IFF_BROADCAST && ifa->ifa_broadaddr) {
                        struct sockaddr_in *broadaddr = (struct sockaddr_in *)ifa->ifa_broadaddr;
                        std::cout << "    (broadcast-capable) Broadcast address: " 
                                  << inet_ntoa(broadaddr->sin_addr) << std::endl;
                    }
                }
            }
        }
        freeifaddrs(interfaces);
    }

    // Switch back to blocking mode for actual packet reception
    fcntl(sock, F_SETFL, flags);

    std::cout << "Listening for UDP packets (including broadcasts) on port " << port << "..." << std::endl;

    // Receive loop
    while (true) {
        char buffer[4096];
        struct sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);

        ssize_t received = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, 
                                   (struct sockaddr*)&sender_addr, &sender_len);

        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "Error receiving packet: " << strerror(errno) << std::endl;
            }
            continue;
        }

        // Null-terminate the buffer
        buffer[received] = '\0';

        // Print the ASCII contents followed by a carriage return
        std::cout << buffer << std::endl;
    }

    close(sock);
    return 1;
}