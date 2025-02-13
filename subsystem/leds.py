from wpilib import AddressableLED, Color, LEDPattern, SmartDashboard
from toolkit.subsystem import Subsystem
import ntcore


class AddressableLEDStrip(Subsystem):

    def __init__(self,
            port: int,
            size: int,
            speed: int,
            brightness: int,
            saturation: int,
            spacing: int,
            # blink_frequency: int,
            # rightlimit,re
            # leftlimit
            ):

        self.size = size
        self.port = port
        # self.speed = speed
        self.brightness = brightness
        self.saturation = saturation
        self.LEDSpacing = spacing
        # self.blink_frequency = blink_frequency #seconds
        # self.rightlimit = rightlimit
        # self.leftlimit = leftlimit
        self.pattern = None
        self.mode="None"


    def init(self):
        """
        initialize new LED strip connected to PWM port
        """
        self.led = AddressableLED(self.port)
        self.led.setLength(self.size)
        self.ledBuffer = [self.led.LEDData() for i in range(self.size)]
        self.enable()
        SmartDashboard.putBoolean("LEDs Initialized", True)

    def enable(self):
        '''
        be sure to enable
        '''
        self.led.start()

    def disable(self):
        self.led.stop()

    def set_brightness(self, brightness: float):
        self.brightness = brightness

    def set_speed(self, speed: float):
        self.speed = speed

    def get_led_data(self):
        return [self.led.LEDData() for i in range(self.size)].copy()

    # def get_current_type(self):
    #     return self.mode

    def set_Solid(self, r: int, g: int, b: int):
        self.pattern = LEDPattern.solid(Color(r, g, b))
        self.mode=f"Solid r:{r} g:{g} b:{b}"

    def set_Alternate(self, r1: int, g1: int, b1: int, r2:int, g2:int, b2: int):

        self.alternate = []
        for i in range(self.size):
            if i % 2 == 0:
                self.alternate.append((i/self.size, Color(r1, g1, b1)))
            elif i % 2 == 1:
                self.alternate.append((i/self.size, Color(r2, g2, b2)))

        self.pattern = LEDPattern.steps(self.alternate)
        self.mode = f"Alternate"


    def set_Rainbow_Ladder(self):
        """
        creates a scrolling rainbow on LEDs
        """
        self.rainbow = LEDPattern.rainbow(self.saturation, self.brightness)

        self.pattern = self.rainbow.scrollAtAbsoluteSpeed(self.speed, self.LEDSpacing)

    def set_Blink(self, r: int, g: int, b: int):

        self.base = LEDPattern.discontinousGradient(r, g, b)
        self.pattern = self.base.blink(self.blink_frequency)

    def field_position(self, r1, g1, b1, r2, b2, g2):
        """
        identify where the robot is on the field

        """
        self.robotposition = 0

        if self.robotposition > self.rightlimit:

            # to do: left side green, right side red

            self.mode = "robot position on starting line is too far right"

        elif self.robotposition < self.leftlimit:

            # to do: left side red, right side green
            self.mode = "robot positioning on starting line is too far left"

        else:
            self.set_solid(0, 100, 0)  # robot is where it needs to be

    def periodic(self):
        self.update_tables()
        def set_pattern_writer(i:int,my_color:Color)->None:
            self.ledBuffer[i].setLED(my_color)
        ledreader=LEDPattern.LEDReader(self.ledBuffer.__getitem__, self.size)
        self.pattern.applyTo(ledreader, set_pattern_writer)
        self.led.setData(self.ledBuffer)

    def update_tables(self):
        self.table=ntcore.NetworkTableInstance.getDefault().getTable("LEDS")
        self.table.putString("mode", self.mode)