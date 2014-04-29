require 'serialport'
require_relative 'utility'
require_relative 'arduino'
require_relative 'leica'
require_relative 'geometry'

class Scanner
  LEICA_PORT = "/dev/tty.DISTOD3903520385-Serial"
  ARDUINO_PORT = "/dev/tty.usbmodem1421"

  MIN_THETA = 30
  MAX_THETA = 150

  MIN_PHI = 0
  MAX_PHI = 180

  INIT_THETA = 113
  INIT_PHI = 153

  def initialize
    @cloud = PointCloud.new

    puts "Initializing Leica..."
    @leica = Leica.new(LEICA_PORT)

    puts "Initializing Arduino..."
    @arduino = Arduino.new(ARDUINO_PORT)

    puts "Devices successfully initialized"

    sleep(1)
  end

  # SIMPLE TEST FUNCTIONS

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

  # Given a phi and theta, finds the plane which that point is on.
  # Assumes that the given scan point is on a flat surface with several degrees
  # of scanning space around it.
  #
  # Returns a Plane object, which is defined by a point and a normal vector.
  def find_plane(init_phi, init_theta, delta_deg)

    phi = init_phi
    theta = init_theta

    @arduino.move(phi, theta)
    r = @leica.measure
    p1 = Point.new({r: r, phi: phi, theta: theta})
    @cloud.add(p1)
    puts "p1: #{p1}"

    phi = init_phi + delta_deg
    theta = init_theta

    @arduino.move(phi, theta)
    r = @leica.measure
    p2 = Point.new({r: r, phi: phi, theta: theta})
    @cloud.add(p2)
    puts "p2: #{p2}"

    phi = init_phi
    theta = init_theta + delta_deg

    @arduino.move(phi, theta)
    r = @leica.measure
    p3 = Point.new({r: r, phi: phi, theta: theta})
    @cloud.add(p3)
    puts "p3: #{p3}"

    plane = Plane.new(p1, p2, p3)

    return plane
  end

  # This method finds the edge of a finite plane in 3d space.
  # Given a point, plane and a vector, it starts at the given point and scans
  # in multiples of a vector from that point, until the scanned point is
  # no longer on the plane. The last scanned point on the plane is returned.
  def fast_find_edge_point(point, plane, vector)
    p = point
    old_p = point
    prev_prev_p = nil
    done = false
    r_increasing = nil # keep track of whether r is increasing as we scan
                       # (used later in the precise scan phase)

    while true # infinite loop broken by return statement
      p = p.add_vector(vector)
      @arduino.move(p.phi, p.theta)
      r = @leica.measure

      if r.nil?
        return nil
      else
        #todo add error handling here
        measured_p = Point.new({r: r, phi: p.phi, theta: p.theta})

        if !plane.include? measured_p
          # start the precise edge search from 1 step behind the last good scan
          precise_start_point = old_p.add_vector(vector * -1)
          return precise_find_edge_point(precise_start_point, vector*0.2, r_increasing)
        else
          if r_increasing.nil?
            # remember whether r is increasing as we scan
            r_increasing = (old_p.r < p.r)
          end
          old_p = p
        end
      end
    end
  end

  # pinpoints an edge by
  def precise_find_edge_point(point, vector, r_increasing)
    scan_p = point
    prev_p = point.add_vector(vector * -1) # guarantee a good scan on first move
    prev_prev_p = prev_p
    done_flag = false

    while true # infinite while broken by a return statement
      # Check the point cache, then do a distance measurement if no cache hit
      p = @cloud.find_point_by_angle(scan_p.phi, scan_p.theta)
      if p.nil?
        @arduino.move(scan_p.phi, scan_p.theta)
        r = @leica.measure
        unless r.nil?
          p = Point.new({r: r, phi: scan_p.phi, theta: scan_p.theta})
          @cloud.add(p)
        end
        puts "measured: #{p}"
      else
        puts "cache hit: #{p}"
      end

      # Detect the change in sign in gradient of r
      wrong_direction = ((r_increasing && p.r < prev_p.r) ||
                         (!r_increasing && p.r > prev_p.r))
      # Detect a sudden large change in r (compared to prev scan or 2 scans ago)
      large_delta = (((p.r - prev_p.r).abs / prev_p.r) > 0.1 ||
                     ((p.r - prev_prev_p.r).abs / prev_prev_p.r) > 0.1)

      # Go two measurements to confirm that the edge detection wasn't a fluke
      if wrong_direction || large_delta
        puts "wrong direction" if wrong_direction
        puts "large delta" if large_delta

        if done_flag
          puts "two scans in a row wrong direction, edge found"
          return prev_prev_p
        else
          puts "opposite direction scanned, done flag set"
          done_flag = true
          prev_prev_p = prev_p
        end
      else
        if done_flag
          puts "next scan was in proper direction, done flag reset"
          done_flag = false
        end
      end

      # keep track of the previous r so we can detect the correct direction to move
      prev_p = p
      scan_p = Point.new({x: p.x + vector[0],
                          y: p.y + vector[1],
                          z: p.z + vector[2]})

    end
  end

  # point the Leica at a given point and activate the laser, to demonstrate
  # the physical location of the point
  def illuminate(point)
    @arduino.move(point.phi, point.theta)
    @leica.measure
  end

  def brute_force_scan(min_theta=MIN_THETA, max_theta=MAX_THETA,
                       min_phi=MIN_PHI, max_phi=MAX_PHI)
    theta = min_theta
    while theta <= max_theta
      phi = min_phi
      while phi <= max_phi
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

  def find_box_1(init_phi, init_theta, delta_deg)
    edge_points = Array.new

    plane = find_plane(init_phi, init_theta, delta_deg)

    puts "SCANNING RIGHT"
    movement_vector = SpatialVector[0, 0, -1].cross_product(plane.normal).normalize * 3
    edge_points[0] = fast_find_edge_point(plane.point, plane, movement_vector)
    puts "edge found: #{edge_points[0]}"

    puts "SCANNING LEFT"
    movement_vector = SpatialVector[0, 0, 1].cross_product(plane.normal).normalize * 3
    edge_points[1] = fast_find_edge_point(plane.point, plane, movement_vector)
    puts "edge found: #{edge_points[1]}"

    puts "SCANNING UP"
    movement_vector = SpatialVector[0, 0, 1].normalize * 3
    edge_points[2] = fast_find_edge_point(plane.point, plane, movement_vector)
    puts "edge found: #{edge_points[2]}"

    puts "SCANNING DOWN"
    movement_vector = SpatialVector[0, 0, -1].normalize * 3
    edge_points[3] = fast_find_edge_point(plane.point, plane, movement_vector)
    puts "edge found: #{edge_points[3]}"

    x_mid = (edge_points[0].x + edge_points[1].x) / 2
    y_mid = (edge_points[0].y + edge_points[1].y) / 2
    z_mid = (edge_points[2].z + edge_points[3].z) / 2

    midpoint = Point.new({x: x_mid, y: y_mid, z: z_mid})
    puts "midpoint: #{midpoint}"

    corner_points = Array.new
    [0, 1].each do |i1|
      [2, 3].each do |i2|
        corner_points << Point.new({x: edge_points[i1].x,
                                    y: edge_points[i1].y,
                                    z: edge_points[i2].z})
      end
    end

    edge_points.each do |p|
      illuminate(p)
    end

    corner_points.each do |p|
      illuminate(p)
    end

    illuminate(midpoint)

  end
end
