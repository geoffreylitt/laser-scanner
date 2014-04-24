class Utility
  def self.deg(rad)
    rad * (180 / Math::PI)
  end

  def self.rad(deg)
    deg * (Math::PI / 180)
  end
end
