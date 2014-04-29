require_relative 'serial_device'

class Leica < SerialDevice
  BAUD_RATE = 9600

  def initialize(port, *params)
    super(port, BAUD_RATE, *params)
    sleep(2)
    @serial.write("a\r") # turn device on, prepare for measurements
    result = @serial.read(3) # read "?\r\n" response from Leica

    # Raise an exception if the Leica errors.
    raise "Leica device error #{result}" if result[0] == '@'
  end

  # Takes a distance measurement and returns the measured distance in m.
  def measure
    # Command the Leica to take a measurement
    @serial.write("g\r")

    result = @serial.read(2) # peek at the first character to test for error
    if result == "@E" #error measuring
      sleep(0.1) # ensure all data has been sent
      @serial.flush_input
      return nil
    elsif result == "31"
      # Read 30 bytes plus \r\n. Blocks until the Leica is done measuring.
      result += @serial.read(32)

      # Process distance result
      dist = result[8..14].to_f
      dist_meters = dist / 10000

      return dist_meters
    else
      raise "Unknown Leica response: #{result + @serial.read}"
    end
  end
end
