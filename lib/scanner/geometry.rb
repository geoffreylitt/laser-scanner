require 'matrix'

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
      @theta = Utility.deg(Math.acos(@z / @r)).round(3)
      @phi = Utility.deg(Math.atan2(@y, @x)).round(3)
      @r = @r / SCALING_FACTOR
    else
      raise ArgumentError, "malformed args hash -- must contain x,y,z or r,phi,theta"
    end
  end

  def to_s
    "x: #{x}, y: #{y}, z: #{z}"
  end

  # Returns a vector from this point to another given point
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

  # Returns a point that is the result of adding vector v to this point
  def add_vector(v)
    return Point.new({x: self.x + v[0],
                      y: self.y + v[1],
                      z: self.z + v[2]})
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

  # Writes an output file that can be imported into MeshLab and other programs
  # coord_sys is either :spherical or :cartesian
  def output(coord_sys)
    File.open("../outfiles/#{coord_sys.to_s}_#{Time.now.strftime("%y%m%d-%H%M%S")}.txt", "w") do |out_file|
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
  ANGLE_TOLERANCE = 4 #experimentally determined to be effective
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
