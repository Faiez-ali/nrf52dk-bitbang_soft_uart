# Project Title

## Project Overview
This project is a software UART implementation for the nRF52 development kit. It allows for bit-banging UART communication on devices that do not have dedicated UART hardware.

## Features
- Bit-bang UART communication
- Adjustable baud rates
- Easy integration with existing projects

## Hardware Requirements
- Nordic nRF52 Development Kit
- LEDs and push buttons for testing

## Software Requirements
- nRF5 SDK
- ARM GCC Toolchain

## Build Instructions
1. Clone the repository:
   ```
   git clone https://github.com/Faiez-ali/nrf52dk-bitbang_soft_uart.git
   ```
2. Navigate to the project directory:
   ```
   cd nrf52dk-bitbang_soft_uart
   ```
3. Compile the project:
   ```
   make
   ```

## Configuration
Modify the configuration parameters in `config.h` to adjust baud rates, pin assignments, and other settings.

## Usage Examples
1. Initialize the UART:
   ```
   soft_uart_init();
   ```
2. Send a character:
   ```
   soft_uart_send('A');
   ```

## Testing
Use the LEDs to verify that signals are being sent and received correctly. Connect to a serial monitor to observe the output.

## Troubleshooting
- Ensure that all hardware connections are correct.
- Verify that the correct COM port is selected in the serial monitor.

## Performance Considerations
- Bit-banging may introduce latency. Adjust the timing parameters for optimal performance.
- Test under varying load conditions to assess the performance impact.