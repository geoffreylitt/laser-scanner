require 'serialport'
require 'matrix'
require './utility'
require './devices'
require './geometry'

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

