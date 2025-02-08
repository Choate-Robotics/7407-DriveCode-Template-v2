from wpilib import AddressableLED, PowerDistribution, SmartDashboard, LEDPattern, Color
import math, config
from utils.local_logger import LocalLogger

class LED_String():
    """
    Addressable LEDs

    """

    def __init__(self, port: int, size: int):

        self.size = size

        self.port = port
        self.log = LocalLogger("LEDs")

        self.speed = 5
        self.brightness = 1
        """ density of *120* per meter"""
        self.LEDSpacing = 1 / 60
        #self.blinkfrequency = 1.5  # seconds
        #self.rightlimit = 100
        #self.leftlimit = 50
        self.led_strip = None
        self.led_buffer = None

    def init(self):
        """
        initialize new LED strip connected to PWM port
        """
        self.led_strip = AddressableLED(self.port)
        self.led_buffer = [AddressableLED.LEDData() for i in range(self.size)]
        self.led_strip.setLength(self.size)
        self.enable()
        """
        create new 
        """
        #self.ledBuffer = self.led.LEDData()
        #self.ledBuffer.setLength(self.size)

        #self.led.setData(self.ledBuffer)

        self.log.setup("LEDs Initialized")

    def enable(self):
        # Working
        self.led_strip.start()

    def disable(self):
        # Working
        self.led_strip.stop()

    def set_brightness(self, brightness: float):
        # No errors, but does it do anything.
        self.brightness = brightness

    def get_led_data(self):
        # No errors, but does it do anything.
        return self.led_buffer.copy()

    def get_current_type(self):
        # Don't understand what this is supposed to do
        if self.mode is None:
            return {
                'type': 0,
                'color': {
                    'r': 0,
                    'g': 0,
                    'b': 0
                }
            }
        return self.mode

    def set_LED(self, brightness: float = 1.0, speed: int = 5):
        self.speed = speed
        self.brightness = brightness

    def set_Solid(self, r: int, g: int, b: int):
        # The one that I think works right now
        self.solid = LEDPattern.solid(Color(r, g, b), self.brightness)
        self.solid.applyTo(self.led_buffer)
        self.mode = "solid"

    def set_Rainbow_Ladder(self):

        self.rainbow = LEDPattern.rainbow(self.speed, self.brightness)
        # scrolls the rainbow
        self.scrollingRainbow = self.rainbow.scrollAtAbsoluteSpeed(self.speed, self.LEDSpacing)
        self.scrollingRainbow.applyTo(self.led_buffer)
        self.mode = "rainbow"

    def set_Blink(self, r, g, b, blinkfrequency):
        self.base = LEDPattern.blink(Color(r, g, b), Color(0, 0, 0), blinkfrequency, self.brightness)
        #self.base = LEDPattern.discontinousGradient(Color(r, g, b), Color(0, 0, 0), self.brightness)
        #self.pattern = self.base
        self.base.applyTo(self.led_buffer)
        #self.led.setData(self.baseledBuffer)

        self.mode = "blink"

    # def field_position(self, r1, g1, b1, r2, b2, g2):
    #     """
    #     identify where the robot is on the field
    #
    #     """
    #     self.robotposition = 0
    #
    #     if self.robotposition > self.rightlimit:
    #
    #         # to do: left side green, right side red
    #
    #         self.mode = "robot position on starting line is too far right"
    #
    #     elif self.robotposition < self.leftlimit:
    #
    #         # to do: left side red, right side green
    #         self.mode = "robot positioning on starting line is too far left"
    #
    #     else:
    #         self.set_solid(0, 100, 0)  # robot is where it needs to be

    def periodic(self):
        self.led_strip.setData(self.led_buffer)
class SLEDS:
    """
    Switchable LEDS from Switchable PDH
    """

    def on(self):
        PowerDistribution.setSwitchableChannel(True)

    def off(self):
        PowerDistribution.setSwitchableChannel(False)