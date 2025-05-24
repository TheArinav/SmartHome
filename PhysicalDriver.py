import time
import RPi.GPIO as GPIO
import pigpio

gpio_mode_set = False
pi = pigpio.pi()

#region PINS

OUT_PIN_LED_BLUE   = 27
OUT_PIN_LED_GREEN  = 22
OUT_PIN_LED_YELLOW = 18
OUT_PIN_LED_ORANGE = 23
OUT_PIN_LED_RED    = 24
OUT_PIN_BTTN_ROW_1 = 25
OUT_PIN_BTTN_ROW_2 = 8
OUT_PIN_BTTN_ROW_3 = 7
OUT_PIN_BTTN_ROW_4 = 26
OUT_PIN_SPEAKER    = 12
OUT_PIN_LED_IR     = 16
IN_PIN_IR          = 20
IN_PIN_BTTN_COL_1  = 21
IN_PIN_BTTN_COL_2  = 4
IN_PIN_BTTN_COL_3  = 17
IN_PIN_SDA         = 2
IN_PIN_SCL         = 3

#endregion

#region GENERAL

def setup_gpio():
    global gpio_mode_set
    if gpio_mode_set:
        return
    GPIO.setmode(GPIO.BCM)
    output_pins = [OUT_PIN_LED_BLUE, OUT_PIN_LED_GREEN, OUT_PIN_LED_YELLOW,
                   OUT_PIN_LED_ORANGE, OUT_PIN_LED_RED, OUT_PIN_SPEAKER]
    for pin in output_pins:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    gpio_mode_set = True
    print("[GPIO] Initialized")

def send_ir_signal(code_hex: str):
    if not pi.connected:
        print("[IR_TX] pigpio not connected")
        return

    try:
        code = int(code_hex, 16)
    except ValueError:
        print(f"[IR ERROR] Invalid hex code: {code_hex}")
        return

    print(f"[IR_TX] Sending NEC code: {code_hex}")

    def nec_encode(code):
        def pulse(on, length):
            return pigpio.pulse(1 << OUT_PIN_LED_IR if on else 0, 0 if on else 1 << OUT_PIN_LED_IR, length)

        sequence = [pulse(True, 9000), pulse(False, 4500)]
        for i in range(32):
            sequence.append(pulse(True, 560))
            if (code >> (31 - i)) & 1:
                sequence.append(pulse(False, 1690))
            else:
                sequence.append(pulse(False, 560))
        sequence.append(pulse(True, 560))
        return sequence

    pi.set_mode(OUT_PIN_LED_IR, pigpio.OUTPUT)
    pi.wave_clear()
    sequence = nec_encode(code)
    pi.wave_add_generic(sequence)
    wid = pi.wave_create()
    if wid >= 0:
        pi.wave_send_once(wid)
        while pi.wave_tx_busy():
            time.sleep(0.01)
        pi.wave_delete(wid)

#endregion