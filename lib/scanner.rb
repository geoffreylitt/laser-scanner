require_relative 'scanner/utility'
require_relative 'scanner/devices'
require_relative 'scanner/geometry'
require_relative 'scanner/scanner'

s = Scanner.new
#s.test_both
#s.test_leica
# s.test_arduino
# s.find_closest
# s.test_plane_finding(153, 113)
#s.test_cartesian_movements
s.find_box_1(153, 113)
