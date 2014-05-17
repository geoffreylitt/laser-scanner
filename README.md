Intelligently Targeted 3-Dimensional Scanning with a Laser Distance Meter
====================
Yale University Senior Project, in partial fulfillment of the requirements for a B.S. Degree in Electrical Engineering and Computer Science

### To use this code, you will need:

- An Arduino (I used an Uno)
- A computer with Bluetooth and USB (I used a Mac, and any Linux computer should work too) with Ruby 2.1.1+ installed (the code uses features of the Ruby Vector library added in 2.1.1)
- A Leica Disto D330i (similar models should also work)
- a pan-tilt mechanism made of two servos.

### Follow these steps to get the project running:

1. Load `./arduino/scan/scan.ino` onto the Arduino and wire up servos to power/ground, and the appropriate control lines -- currently pin 9 for the horizontal servo and pin 10 for the vertical servo.
2. connect a Leica Disto via Bluetooth to the computer and establish a Bluetooth serial connection (this setup varies depending on the OS). Change the hardcoded port name in `./lib/scanner/scanner.rb` to match the Leica connection port.
3. Uncomment one of the lines in `./lib/scanner.rb` to choose a scan function to use. See `./lib/scanner/scanner.rb` for explanations of what each method does.
5. `ruby lib/scanner.rb`
