Yale EECS Senior Project - Intelligently Targeted 3-Dimensional Scanning with a Laser Distance Meter
====================
Geoffrey Litt '14
--------------------

Arduino code is contained in ./arduino
Ruby code is contained in ./lib

To use this code, you need to:
- load arduino/scan/scan.ino onto an Arduino and connect servos to the right pins,
  and verify the Arduino port name
- connect a Leica Disto via Bluetooth to the computer and verify the port name
- uncomment a line in lib/scanner.rb to choose a scan function to use.
- Ensure you're using Ruby 2.1.1+ (the code uses a Vector function added in 2.1.1)
- `cd lib`
- `ruby scanner.rb`
