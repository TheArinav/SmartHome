# PhysicalDriver.py

# TODO: Replace these with actual GPIO/I2C/hardware control code
def send_ir_signal(code: str):
    print(f"[IR] Sending IR code: {code}")
    # TODO: Use LIRC or pigpio to send actual IR command

def display_text(text: str):
    print(f"[LCD] Display: {text}")
    # TODO: Write to I2C LCD screen

def set_led(color: str):
    print(f"[LED] Set color: {color}")
    # TODO: Set GPIO pin for the specific LED

def play_buzzer(sound_type: str):
    print(f"[Buzzer] Sound: {sound_type}")
    # TODO: Output tone via PWM pin

# Action dispatcher
def perform_action(action: str):
    """Parses and dispatches an action string like 'ir:0xAABBCCDD' or 'led:green'."""
    try:
        kind, value = action.split(":", 1)
    except ValueError:
        print(f"[ERROR] Invalid action format: {action}")
        return

    if kind == "ir":
        send_ir_signal(value)
    elif kind == "display":
        display_text(value)
    elif kind == "led":
        set_led(value)
    elif kind == "buzzer":
        play_buzzer(value)
    else:
        print(f"[WARN] Unknown action type: {kind}")
