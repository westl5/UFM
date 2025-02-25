#include <windows.h>
#include <iostream>
#include <string>

class SerialPort {
public:
    SerialPort(const std::string& portName, DWORD baudRate = CBR_9600)
        : hSerial(INVALID_HANDLE_VALUE) {
        open(portName, baudRate);
    }

    ~SerialPort() {
        close();
    }

    bool isOpen() const {
        return hSerial != INVALID_HANDLE_VALUE;
    }

    std::string read() {
        if (!isOpen()) return "";

        DWORD bytesRead;
        char buffer[256];
        if (ReadFile(hSerial, buffer, sizeof(buffer), &bytesRead, NULL)) {
            return std::string(buffer, bytesRead);
        }
        return "";
    }

private:
    HANDLE hSerial;

    void open(const std::string& portName, DWORD baudRate) {
        hSerial = CreateFile(portName.c_str(), GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (hSerial == INVALID_HANDLE_VALUE) {
            std::cerr << "Error opening serial port!" << std::endl;
            return;
        }

        DCB dcbSerialParams = { 0 };
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Error getting serial port state!" << std::endl;
            close();
            return;
        }

        dcbSerialParams.BaudRate = baudRate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;
        if (!SetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Error setting serial port state!" << std::endl;
            close();
            return;
        }

        COMMTIMEOUTS timeouts = { 0 };
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        if (!SetCommTimeouts(hSerial, &timeouts)) {
            std::cerr << "Error setting serial port timeouts!" << std::endl;
            close();
            return;
        }
    }

    void close() {
        if (isOpen()) {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
        }
    }
};

int main() {
    std::string portName = "\\\\.\\COM3"; // Change this to your serial port
    SerialPort serial(portName);

    if (!serial.isOpen()) {
        return 1;
    }

    std::cout << "Monitoring serial port " << portName << ". Press Ctrl+C to exit." << std::endl;

    while (true) {
        std::string data = serial.read();
        if (!data.empty()) {
            std::cout << data;
        }
    }

    return 0;
}