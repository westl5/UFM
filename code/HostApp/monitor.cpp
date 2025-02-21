#include <windows.h>
#include <iostream>
#include <thread>
#include <atomic>

// Atomic flag to signal when to stop reading
std::atomic<bool> keepReading(true);

// Thread function to read from the serial port asynchronously
void ReadThread(HANDLE hSerial) {
    const DWORD bufferSize = 256;
    char buffer[bufferSize] = {0};
    DWORD bytesRead = 0;

    // Set up an OVERLAPPED structure for asynchronous I/O.
    OVERLAPPED osReader = {0};
    osReader.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (osReader.hEvent == NULL) {
        std::cerr << "Error creating overlapped event." << std::endl;
        return;
    }

    while (keepReading.load()) {
        // Clear the buffer for new data.
        ZeroMemory(buffer, bufferSize);
        bytesRead = 0;

        // Initiate an overlapped read.
        if (!ReadFile(hSerial, buffer, bufferSize - 1, &bytesRead, &osReader)) {
            if (GetLastError() != ERROR_IO_PENDING) {
                std::cerr << "ReadFile failed with error: " << GetLastError() << std::endl;
                break;
            } else {
                // Wait for the asynchronous read to complete with a 100ms timeout.
                DWORD dwRes = WaitForSingleObject(osReader.hEvent, 100);
                if (dwRes == WAIT_OBJECT_0) {
                    if (!GetOverlappedResult(hSerial, &osReader, &bytesRead, FALSE)) {
                        std::cerr << "GetOverlappedResult failed with error: " << GetLastError() << std::endl;
                        break;
                    }
                } else if (dwRes == WAIT_TIMEOUT) {
                    // Timeout; continue to check if we should keep reading.
                    continue;
                } else {
                    std::cerr << "WaitForSingleObject failed with error: " << GetLastError() << std::endl;
                    break;
                }
            }
        }

        // If data was received, print it.
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';  // Null-terminate the string
            std::cout << "Received: " << buffer << std::endl;
        }
    }

    CloseHandle(osReader.hEvent);
}

int main() {
    // Specify the COM port (use "\\\\.\\COM3" format for COM ports)
    const char* portName = "\\\\.\\COM3";
    HANDLE hSerial = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                                 OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "Error opening serial port " << portName << ". Error code: " << GetLastError() << std::endl;
        return 1;
    }

    // Configure the serial port parameters
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Error getting current serial parameters." << std::endl;
        CloseHandle(hSerial);
        return 1;
    }

    // Set desired serial port settings (adjust as needed)
    dcbSerialParams.BaudRate = CBR_115200;  // Baud rate
    dcbSerialParams.ByteSize = 8;           // Data bits: 8
    dcbSerialParams.StopBits = ONESTOPBIT;  // One stop bit
    dcbSerialParams.Parity   = NOPARITY;    // No parity
    if (!SetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Error setting serial port parameters." << std::endl;
        CloseHandle(hSerial);
        return 1;
    }

    // Set communication timeouts (these can be adjusted for responsiveness)
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    if (!SetCommTimeouts(hSerial, &timeouts)) {
        std::cerr << "Error setting timeouts." << std::endl;
        CloseHandle(hSerial);
        return 1;
    }

    // Launch a dedicated thread for reading data
    std::thread reader(ReadThread, hSerial);

    std::cout << "Serial monitor running on " << portName << ". Press Enter to exit." << std::endl;
    std::cin.get(); // Wait for user input to exit

    // Signal the reader thread to stop and wait for it to finish
    keepReading.store(false);
    reader.join();

    CloseHandle(hSerial);
    return 0;
}
