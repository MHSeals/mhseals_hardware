from machine import Pin
import time

LED_PIN = Pin("LED", Pin.OUT)
GPIO_PINS = [11, 12, 13, 14]
RUN_SECONDS = 30

def main():
    try:
        print(f"Running pins {GPIO_PINS} high for {RUN_SECONDS} s...")
        LED_PIN.on()
        
        for i in range(20):
            pin = Pin(i, Pin.OUT)
            pin.value(1)

        time.sleep(RUN_SECONDS)
        
    finally:    
        for i in range(20):
            pin = Pin(i, Pin.OUT)
            pin.value(0)

        print("Done — all pins set to low.")
        LED_PIN.off()
        
if __name__ == "__main__":
    main()