from machine import Pin, PWM
import time

LED_PIN = Pin("LED", Pin.OUT)

GPIO_PINS = [11, 12, 13, 14]
FREQUENCY = 50
RUN_SECONDS = 5
MAX_DUTY = 65535

NEUTRAL_US = 1500
FORWARD_US = 2000
PERIOD_US = 1000000 // FREQUENCY

def main():
    try:
        print(f"Running ESCs on GPIO {GPIO_PINS} at {FREQUENCY} Hz for {RUN_SECONDS} s...")
        LED_PIN.on()
        
        pwm_channels = []
        for gpio in GPIO_PINS:
            pwm = PWM(Pin(gpio))
            pwm.freq(FREQUENCY)
            pwm.duty_u16(int(NEUTRAL_US / PERIOD_US * MAX_DUTY))
            pwm_channels.append(pwm)
        
        time.sleep(2)
        
        for pwm in pwm_channels:
            pwm.duty_u16(int(FORWARD_US / PERIOD_US * MAX_DUTY))
        
        time.sleep(RUN_SECONDS)
        
    finally:    
        for pwm in pwm_channels:
            pwm.duty_u16(int(NEUTRAL_US / PERIOD_US * MAX_DUTY))
            pwm.deinit()
        
        print("Done — all PWM channels stopped.")
        LED_PIN.off()
        
if __name__ == "__main__":
    main()