require_relative 'serial_device'

class Arduino < SerialDevice
  INIT_PHI = 180
  INIT_THETA = 150
  BAUD_RATE = 9600

  MIN_PHI = 0
  MAX_PHI = 180

  MIN_THETA = 0
  MAX_THETA = 150

  # limits on delay between scans
  MIN_DELAY = 2
  MAX_DELAY = 4

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

    # compensate for drifting due to weight imbalance
    # theta = theta - (4 * theta / 90)
    theta = theta - 5

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
    sleep_time = [[(0.1 * delta), MAX_DELAY].min, MIN_DELAY].max

    # helps eliminate servo "drift" issues
    # @phi = INIT_PHI
    # @theta = INIT_THETA
    # transmit

    # sleep(0.5)

    @phi = phi
    @theta = theta
    transmit

    # sleep(sleep_time)
    sleep(2)

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
