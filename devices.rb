class SerialDevice
  attr_accessor :port
  def initialize(port, *params)
    @serial = SerialPort.new(port, *params)
  end
end

class Arduino < SerialDevice
  INIT_PHI = 180
  INIT_THETA = 150
  BAUD_RATE = 9600

  MIN_PHI = 0
  MAX_PHI = 180

  MIN_THETA = 0
  MAX_THETA = 150

  attr_accessor :phi, :theta

  # initialize the serial connection and initial angles
  def initialize(port, *params)
    super(port, BAUD_RATE, *params)
    @phi = INIT_PHI
    @theta = INIT_THETA
    sleep(3)
    move(@phi, @theta)
  end

  # given a phi and theta, moves the pan-tilt to that angle
  def move(phi, theta)
    if (phi < MIN_PHI or phi > MAX_PHI)
      raise ArgumentError, "invalid phi"
    end

    if (theta < MIN_THETA or theta > MAX_THETA)
      raise ArgumentError, "invalid theta"
    end

    # track largest angular delta, for sleeping appropriate amount later
    d_phi = (@phi - phi).abs
    d_theta = (@theta - theta).abs
    delta = [d_phi, d_theta].max
    sleep_time = [[(0.05 * delta), 2].min, 1].max

    @phi = phi
    @theta = theta
    transmit

    sleep(sleep_time)

  end

  private

  # These two methods are used (if nec.) for scaling phi and theta values
  # in our external coordinate system to angles which are sent
  # to the servos. They also add padding zeros to follow the comm
  # protocol with the Arduino.

  def scaled_phi
    "%03d" % @phi
  end

  def scaled_theta
    "%03d" % @theta
  end

  # Transmit angle values to Arduino
  def transmit
    out = "#{scaled_phi};#{scaled_theta}\n"
    @serial.write(out)

    #wait for ack from Arduino
    @serial.read(3) #"OK\n"
  end

end

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
