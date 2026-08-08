from machine import Pin, PWM
import time

LED_PIN = Pin("LED", Pin.OUT)
ESTOP_PIN = Pin(15, Pin.OUT)

GPIO_PINS = [11, 12, 13, 14]
FREQUENCY = 50
WAIT_SECONDS = 1
NEUTRAL_SECONDS = 5
RUN_SECONDS = 5
MAX_DUTY = 65535

NEUTRAL_US = 1300
FORWARD_US = 1700
PERIOD_US = 1000000 // FREQUENCY

def main():
    try:
        print(f"Running ESCs on GPIO {GPIO_PINS} at {FREQUENCY} Hz for {RUN_SECONDS} s...")

        LED_PIN.on()
        ESTOP_PIN.on()
        
        print(f"Waiting for {WAIT_SECONDS} s...")
        time.sleep(WAIT_SECONDS)
        
        pwm_channels = []
        for gpio in GPIO_PINS:
            pwm = PWM(Pin(gpio))
            pwm.freq(FREQUENCY)
            pwm.duty_u16(int(NEUTRAL_US / PERIOD_US * MAX_DUTY))
            pwm_channels.append(pwm)
        
        print(f"Sending neutral command at pulse width {NEUTRAL_US} for {NEUTRAL_SECONDS} s")
        time.sleep(NEUTRAL_SECONDS)
        print(f"Sending forward command at pulse width {FORWARD_US} for {RUN_SECONDS} s")
        
        for pwm in pwm_channels:
            pwm.duty_u16(int(FORWARD_US / PERIOD_US * MAX_DUTY))
        
        time.sleep(RUN_SECONDS)
        
    finally:    
        for pwm in pwm_channels:
            pwm.duty_u16(int(NEUTRAL_US / PERIOD_US * MAX_DUTY))
            pwm.deinit()
        
        print("Done — all PWM channels stopped.")
        LED_PIN.off()
        ESTOP_PIN.off()
        
if __name__ == "__main__":
    main()