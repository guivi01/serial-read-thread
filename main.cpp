// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>

class serial
{
private:
    int m_fdPort;
    std::string m_strPort;
    std::mutex m_fdMutex;
    std::thread m_tRead;
    bool m_listening;

    std::thread _listening() {
          return std::thread([=] { listening(); });
    };
    void listening()
    {
        m_listening = true;
        while (m_listening)
        {
            fd_set reads;
            FD_ZERO(&reads);
            FD_SET( m_fdPort, &reads);
            struct timeval timout;
            timout.tv_sec = 0;
            timout.tv_usec = 100000;
            if (select(m_fdPort+1, &reads, 0, 0, &timout) < 0)
            {
                m_listening = false;
                return;
            }
            if (FD_ISSET( m_fdPort, &reads)) {
                char read_buf[4096];
                memset(&read_buf, '\0', sizeof(read_buf));
                int bytes_received = read(m_fdPort, &read_buf, sizeof(read_buf));
                if (bytes_received > 0) {
                    printf("Received (%d bytes): %.*s",
                        bytes_received, bytes_received, read);
                }                
            }
        }
    };
public:
    serial()
    : m_fdPort(-1)
    , m_strPort("/dev/ttyUSB0")
    , m_listening(false)
    {
    };
    ~serial()
    {

    };
    bool open_port(std::string port)
    {
        if (is_opened())
            return false;
        // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
        m_fdPort = open(port.c_str(), O_RDWR);
        // Create new termios struct, we call it 'tty' for convention
        struct termios tty;
        // Read in existing settings, and handle any error
        if(tcgetattr(m_fdPort, &tty) != 0) {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
            return false;
        }
        tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
        tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
        tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
        tty.c_cflag |= CS8; // 8 bits per byte (most common)
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
        // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

        tty.c_cc[VTIME] = 5;    // Wait for up to 10s (100 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        // Set in/out baud rate to be 9600
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);

        // Save tty settings, also checking for error
        if (tcsetattr(m_fdPort, TCSANOW, &tty) != 0) {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
            return false;
        }
        return true;
    };
    bool is_opened()
    {
        return (m_fdPort >= 0) ? true : false;
    };
    void start_listening()
    {
        if (!is_opened())
            return;
        if (m_listening)
            return;        
        m_tRead = _listening();
    };
    int send_date(std::string data)
    {
        if (m_fdPort >= 0)
        {
            return write(m_fdPort, data.c_str(), data.size());
        }
        return 0;
    }
};

int main() 
{
    serial port;
    if (!port.open_port("/dev/serial0"))
    {   
        std::cout << "Could not open the serial port" << std::endl;
        return 1;
    }

    while (true)
    {
        std::string str;
        std::cin >> str;
        port.send_date(str);
    }
};
