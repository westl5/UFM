#include "pa_driver/pa_driver.hpp"
#include <fstream>
#include <iostream>

int main(int argc, char* argv[]) {
    // Open recording file
    std::ifstream recording("paddle0.bin", std::ios::binary);
    if (!recording.is_open()) {
        std::cerr << "Failed to open paddle0.bin\n";
        return 1;
    }

    // Read and process recorded data
    PA_object objects[16];
    while (recording.read(reinterpret_cast<char*>(objects), sizeof(objects))) {
        // Process the objects here
        for (const auto& obj : objects) {
            if (obj.area > 0) {
                std::cout << "Object: x=" << obj.center_x << " y=" << obj.center_y 
                         << " area=" << obj.area << "\n";
            }
        }
    }

    return 0;
} 