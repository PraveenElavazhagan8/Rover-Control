import board
import busio
import adafruit_pca9685
import time 
from adafruit_servokit import ServoKit

try:
    kit = ServoKit(channels=16)
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = adafruit_pca9685.PCA9685(i2c)
    pca.frequency = 60
    relay_channel = 0

    # Calculate duty cycle for the pulse width of 1700
    pulse_width = 1900  # in microseconds
    pulse_width_in_seconds = pulse_width / 1_000_000  # convert to seconds
    duty_cycle = int(pulse_width_in_seconds * pca.frequency * 4096)
    
    print(f"activating relay on channel {relay_channel}")
    pca.channels[relay_channel].duty_cycle = duty_cycle
    time.sleep(5)

    print(f"Deactivating relay on channel {relay_channel}")
    pca.channels[relay_channel].duty_cycle = 0
   
    pca.deinit()
      
except Exception as e:
    print('exception :{}',format(e))
