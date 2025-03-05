#include <windows.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

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
        char buffer[16];
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

// Function to split a string by a delimiter (e.g., comma)
std::vector<std::string> split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// Function to move the console cursor to a specific position
void moveCursor(int x, int y) {
    if (SetCursorPos(x, y)) {
        ;
    }
    else {
        std::cerr << "Failed to move cursor!" << std::endl;
    }
}

int main() {
    std::string portName = "\\\\.\\COM3"; // Change this to your serial port
    SerialPort serial(portName);

    if (!serial.isOpen()) {
        return 1;
    }

    std::cout << "Monitoring serial port " << portName << ". Press Ctrl+C to exit." << std::endl;

    int lineCount = 0; // Counter for lines
    std::string buffer; // Buffer to accumulate data

    while (true) {
        std::string data = serial.read();
        if (!data.empty()) {
            buffer += data; // Append incoming data to the buffer

            // Check for newline characters in the buffer
            size_t pos;
            while ((pos = buffer.find('\n')) != std::string::npos) {
                std::string line = buffer.substr(0, pos); // Extract the line
                buffer.erase(0, pos + 1); // Remove the processed line from the buffer

                lineCount++; // Increment the line counter

                // Split the line by commas
                std::vector<std::string> values = split(line, ',');
                if (values.size() >= 2) {
                    // Extract the first two values as coordinates
                    int x = std::stoi(values[0]); // Convert first value to integer (X coordinate)
                    int y = std::stoi(values[1]); // Convert second value to integer (Y coordinate)

                    float res_x = (double)x / 4095.0 * 1920.0;
                    float res_y = (4095.0 - (double)y) / 4095.0 * 1080.0;
                    std::cout << "x raw" << x << "y raw" << y << "\n";
                    std::cout << "x position" << res_x << "y positions" << res_y << "\n";
                    // Move the cursor to the specified position
                    moveCursor(res_x, res_y);

                    // Print a message at the new cursor position
                    //std::cout << "X: " << x << ", Y: " << y;
                }
                else {
                    std::cerr << "Line " << lineCount << ": Invalid format (expected at least 2 values)" << std::endl;
                }
            }
        }
    }

    return 0;
}