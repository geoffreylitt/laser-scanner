class SerialDevice
  attr_accessor :port
  def initialize(port, *params)
    @serial = SerialPort.new(port, *params)
  end
end
