require 'serialport'
require 'matrix'

class Utility
  def self.deg(rad)
    rad * (180 / Math::PI)
  end

  def self.rad(deg)
    deg * (Math::PI / 180)
  end
end

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

class Point
  SCALING_FACTOR = 100
  DECIMALS = 2

  attr_accessor :x, :y, :z, :r, :phi, :theta

  # Point can be constructed from spherical or cartesian coordinates.
  # A hash is passed in with keys (r, phi, theta) or (x, y, z)
  def initialize(args)
    if args.has_key?(:r) and args.has_key?(:phi) and args.has_key?(:theta)
      @r = args[:r]
      @phi = args[:phi]
      @theta = args[:theta]

      f_val = SCALING_FACTOR * @r * Math.sin(Utility.rad(@theta)) * Math.cos(Utility.rad(@phi))
      i_val = f_val.round(DECIMALS)
      @x = i_val

      f_val = SCALING_FACTOR * @r * Math.sin(Utility.rad(@theta)) * Math.sin(Utility.rad(@phi))
      i_val = f_val.round(DECIMALS)
      @y = i_val

      f_val = SCALING_FACTOR * @r * Math.cos(Utility.rad(@theta))
      i_val = f_val.round(DECIMALS)
      @z = i_val
    elsif args.has_key?(:x) and args.has_key?(:y) and args.has_key?(:z)
      @x = args[:x]
      @y = args[:y]
      @z = args[:z]

      @r = Math.sqrt(x**2 + y**2 + z**2).round(5)
      @theta = Utility.deg(Math.acos(@z / @r)).round(0)
      @phi = Utility.deg(Math.atan2(@y, @x)).round(0)
    else
      raise ArgumentError, "malformed args hash -- must contain x,y,z or r,phi,theta"
    end
  end

  def to_s
    "x: #{x}, y: #{y}, z: #{z}"
  end

  def vector_to(p)
    dx = p.x - self.x
    dy = p.y - self.y
    dz = p.z - self.z
    vector = SpatialVector[dx, dy, dz]
    return vector
  end

  def ==(p)
    self.x == p.x && self.y == p.y && self.z == p.z
  end
end

class SpatialVector < Vector
  # Find the angle between this vector and another vector, using the dot product
  def angle_with(v2)
    cos_theta = (self.inner_product(v2)) / (self.r * v2.r)
    theta = Utility.deg(Math.acos(cos_theta))
    return theta
  end
end

class PointCloud
  def initialize
    @points = Array.new
  end

  def add(point)
    @points << point
  end

  def count
    @points.size
  end

  def find_point_by_angle(phi, theta)
    @points.each do |p|
      if p.phi == phi and p.theta == theta
        return p
      end
    end

    return nil
  end

  # coord_sys is either :spherical or :cartesian
  def output(coord_sys)
    File.open("./outfiles/#{coord_sys.to_s}_#{Time.now.strftime("%y%m%d-%H%M%S")}.txt", "w") do |out_file|
      @points.each do |p|
        if coord_sys == :cartesian
          puts "%.4f,%.4f,%.4f" % [p.x, p.y, p.z]
          out_file.puts "%4f,%4f,%4f" % [p.x, p.y, p.z]
        elsif coord_sys == :spherical
          puts "%.4f,%d,%d" % [p.r, p.phi, p.theta]
          out_file.puts "%.4f,%d,%d" % [p.r, p.phi, p.theta]
        else
          raise ArgumentError, "Invalid coordinate system #{coord_sys.to_s}"
        end
      end
    end
  end
end

# A Plane object represents a plane in 3d space.
# It's defined by a point and a normal vector.
class Plane
  ANGLE_TOLERANCE = 3
  MIN_DIST = 10

  attr_accessor :vec_on_plane, :point, :normal

  # Constructs a new Plane object from 3 Point objects.
  def initialize(p1, p2, p3)
    v1 = p1.vector_to(p2)
    v2 = p1.vector_to(p3)
    normal = v1.cross_product(v2)

    @point = p1
    @normal = normal

    # keep around an arbitrary vector on the plane for convenience later
    @vec_on_plane = v1
  end

  # Test whether a given point is on this plane
  def include?(p)
    # We need some distance between the given point and the point on the plane,
    # to get an accurate vector reading. Add a vector on the plane to the
    # given point, until it's suitably far.
    while @point.vector_to(p).r < MIN_DIST
      puts "adding distance between query point and reference point"
      p.x += @vec_on_plane[0]
      p.y += @vec_on_plane[1]
      p.z += @vec_on_plane[2]
    end

    # Create a vector from a point on the plane to the given point
    plane_v = @point.vector_to(p)

    # Test whether the constructed vector is perpendicular to the plane's normal
    # within a tolerance of +/- ANGLE_TOLERANCE degrees.
    angle = plane_v.angle_with(@normal)
    puts "angle to normal: #{angle}"

    if ((angle - 90).abs < ANGLE_TOLERANCE) || (p == @point)
      return true
    else
      return false
    end
  end
end

class Scanner
  MIN_THETA = 120
  MAX_THETA = 150

  MIN_PHI = 90
  MAX_PHI = 180

  INIT_THETA = 118
  INIT_PHI = 162

  STEP_SIZE = 1

  # degrees to move from starting point to establish plane
  DELTA_DEG = 5

  LEICA_PORT = "/dev/tty.DISTOD3903520385-Serial"
  ARDUINO_PORT = "/dev/tty.usbmodem1421"

  def initialize
    @cloud = PointCloud.new

    puts "Initializing Leica..."
    @leica = Leica.new(LEICA_PORT)

    puts "Initializing Arduino..."
    @arduino = Arduino.new(ARDUINO_PORT)

    puts "Devices successfully initialized"

    sleep(1)
  end

  def test_leica
    while true
      print "Press any key to measure"
      gets.chomp # block on user input
      puts "Distance: #{@leica.measure}m"
    end
  end

  def test_arduino
    while true
      puts "Enter phi: "
      phi = gets.chomp.to_i
      puts "Enter theta: "
      theta = gets.chomp.to_i
      @arduino.move(phi, theta)
    end
  end

  def test_both
    theta = MIN_THETA
    while theta <= MAX_THETA
      phi = MIN_PHI
      while phi <= MAX_PHI
        @arduino.move(phi, theta)
        r = @leica.measure
        unless r.nil?
          p = Point.new({r: r, phi: phi, theta: theta})
          @cloud.add(p)
          puts p
        end
        phi += STEP_SIZE
      end
      theta += STEP_SIZE
    end

    @cloud.output :spherical
    @cloud.output :cartesian
  end

  def find_local_min(dim, phi_init, theta_init)

    raise ArgumentError "invalid dimension" unless [:phi, :theta].include? dim

    phi = phi_init
    theta = theta_init
    done = false
    reverse_flag = false

    min_p = Point.new({r: Float::INFINITY, phi: 0, theta: 0})
    prev_r = 0
    dir = 1

    while true # infinite while broken by a return statement
      # Check the point cache, then do a distance measurement if no cache hit
      p = @cloud.find_point_by_angle(phi, theta)
      if p.nil?
        @arduino.move(phi, theta)
        r = @leica.measure
        unless r.nil?
          p = Point.new({r: r, phi: phi, theta: theta})
          @cloud.add(p)
        end
        puts "measured: #{p}"
      else
        r = p.r
        puts "cache hit: #{p}"
      end

      # movement logic
      # If r increases, we're going the wrong direction along the gradient.
      # The first time it happens, we set a flag to prepare to end the scan.
      # We don't immediately switch directions in case of minor deviations.
      # The second time it happens we know that we're going the wrong direction.
      if(r >= prev_r)
        if reverse_flag
          puts "scan direction reversed"
          dir = dir * -1
          reverse_flag = false
        else
          puts "reverse flag set"
          reverse_flag = true
        end
      # otherwise, we're moving the right direction
      else
        # clear the reverse flag
        puts "reverse flag cleared" if reverse_flag == true
        reverse_flag = false

        # because we assume we start on a monotonic surface, we usually will see
        # the smallest r yet when we move the right direction. in that case,
        # update our record of the minimum r we've seen.
        if(r <= min_p.r)
          min_p = p
        # if the r we scan is not smaller than the minimum, that means we've
        # reached an edge. return the minimum r we scanned.
        else
          return min_p
        end
      end

      # keep track of the previous r so we can detect the correct direction to move
      prev_r = r

      #move specified dimension in the specified direction
      if dim == :phi
        phi += (dir * STEP_SIZE)
      elsif dim == :theta
        theta += (dir * STEP_SIZE)
      end
    end
  end

  def find_closest
    puts "FINDING MIN PHI"
    phi_min = find_local_min(:phi, INIT_PHI, INIT_THETA).phi

    puts "FINDING MIN THETA"
    theta_min = find_local_min(:theta, phi_min, INIT_THETA).theta

    puts "minimum distance point: #{phi_min}, #{theta_min}"

    @arduino.move(phi_min, theta_min)
    sleep(1)
    3.times do
      @leica.measure
    end
  end

  # Given a phi and theta, finds the plane which that point is on.
  # Assumes that the given scan point is on a flat surface with several degrees
  # of scanning space around it.
  #
  # Returns a Plane object, which is defined by a point and a normal vector.
  def find_plane_from(init_phi, init_theta)

    phi = init_phi
    theta = init_theta

    @arduino.move(phi, theta)
    r = @leica.measure
    p1 = Point.new({r: r, phi: phi, theta: theta})
    @cloud.add(p1)
    puts "p1: #{p1}"

    phi = init_phi + DELTA_DEG
    theta = init_theta

    @arduino.move(phi, theta)
    r = @leica.measure
    p2 = Point.new({r: r, phi: phi, theta: theta})
    @cloud.add(p2)
    puts "p2: #{p2}"

    phi = init_phi
    theta = init_theta + DELTA_DEG

    @arduino.move(phi, theta)
    r = @leica.measure
    p3 = Point.new({r: r, phi: phi, theta: theta})
    @cloud.add(p3)
    puts "p3: #{p3}"

    plane = Plane.new(p1, p2, p3)

    puts "In plane: #{plane.include?(p2)}"

    p = p1
    3.times do
      new_p = Point.new({x: p.x + plane.vec_on_plane[0],
                         y: p.y + plane.vec_on_plane[1],
                         z: p.z + plane.vec_on_plane[2]})
      p = new_p
      "moving to #{p.phi}, #{p.theta}"
      @arduino.move(p.phi, p.theta)
      @leica.measure
    end

=begin
    while true
      print "phi: "
      phi = gets.chomp.to_i
      print "theta: "
      theta = gets.chomp.to_i

      @arduino.move(phi, theta)
      r = @leica.measure
      p = Point.new({r: r, phi: phi, theta: theta})
      @cloud.add(p)
      puts "point: #{p}"
      puts "In plane: #{plane.include?(p)}"
    end
=end
  end

  def test_cartesian_movements
    @arduino.move(162, 127)
    r = @leica.measure
    p = Point.new({r: r, phi: 162, theta: 118})
    puts p

    p2 = Point.new({x: p.x + 1, y: p.y + 1, z: p.z})
    puts p2
    puts "moving to #{p2.phi}, #{p2.theta}"
    @arduino.move(p2.phi, p2.theta)
    @leica.measure
  end
end


s = Scanner.new
#s.test_both
#s.test_leica
#s.test_arduino
#s.find_closest
s.find_plane_from(162, 118)
#s.test_cartesian_movements

