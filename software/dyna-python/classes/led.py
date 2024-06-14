import RPi.GPIO as GPIO


class Led:
    """
    The led class handles everything that has to do with an LED
    connected to the raspberry GPIO
    """
    
    def __init__(self, pin):
        """
        When initializing the class a pin must be selected as the 
        GPIO output pin for the led
        """
        GPIO.setmode(GPIO.BCM)
        self.pin = pin
        self.state = False
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, GPIO.LOW)
    

    def turn_on(self):
        GPIO.output(self.pin, GPIO.HIGH)
        self.state = True
    

    def turn_off(self):
        GPIO.output(self.pin, GPIO.LOW)
        self.state = False
    
    
    def toggle(self):
        if self.state:
            GPIO.output(self.pin, GPIO.LOW)
            self.state = False
        else:
            GPIO.output(self.pin, GPIO.HIGH)
            self.state = True
    
    
    def close_led(self):
        GPIO.cleanup(self.pin)
