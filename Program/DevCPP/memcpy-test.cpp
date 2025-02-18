#include <iostream>
#include <cstdint>
#include <cstring> // For memcpy

int main() {
    double var1 = 102.55; // Example double variable

    // Create a uint8_t array with enough space to hold the double variable
    uint8_t var2[8];

    // Use memcpy to copy the bytes of var1 into the uint8_t array
    std::memcpy(var2, &var1, sizeof(double));

    // Output the original double variable
    std::cout << "Original double variable: " << var1 << std::endl;

    // Output the bytes in the uint8_t array
    std::cout << "Bytes in uint8_t array var2: ";
    for (size_t i = 0; i < sizeof(var2); ++i) {
        std::cout << std::hex << +var2[i]; // +var2[i] converts uint8_t to unsigned int for cout
    }
    std::cout << std::endl;
    
    // Create a double variable var3

    double var3;

    // Use memcpy to copy the bytes from var2 into var3
    std::memcpy(&var3, var2, sizeof(double));

    // Output the reconstructed double variable var3
    std::cout << "Reconstructed double variable var3: " << var3 << std::endl;

    return 0;
}