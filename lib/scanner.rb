require 'serialport'
require 'matrix'
require_relative 'scanner/utility'
require_relative 'scanner/devices'
require_relative 'scanner/geometry'
require_relative 'scanner/scanner'

s = Scanner.new
#s.test_both
#s.test_leica
#s.test_arduino
#s.find_closest
s.find_plane_from(162, 118)
#s.test_cartesian_movements
