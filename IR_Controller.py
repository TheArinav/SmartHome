# IR_Controller.py

from PhysicalDriver import (
    perform_action,
    wait_for_button,
    wait_for_ir_signal,
    init_button_matrix,
    setup_gpio,
    lcd_init
)

import time

# DFA Definition
DFA = {
    "idle": {
        "on_enter": ["display:Idle", "led:green"],
        "transitions": {
            "Button:2": "turn_on_tv",
            "Remote:Power": "turn_on_ac",
            "IR_Received:0x00FF30CF": "handle_tv_remote"
        }
    },
    "turn_on_tv": {
        "on_enter": ["ir:0xAABBCCDD", "display:TV ON", "led:blue", "buzzer:short"],
        "transitions": {
            "Button:#": "idle"
        }
    },
    "turn_on_ac": {
        "on_enter": ["ir:0x11223344", "display:AC ON", "led:orange", "buzzer:short"],
        "transitions": {
            "Button:*": "idle"
        }
    },
    "handle_tv_remote": {
        "on_enter": ["display:Remote Control", "led:orange"],
        "transitions": {
            "Button:0": "idle"
        }
    }
}

# Internal state
current_state = "idle"

def handle_command(event: str) -> str:
    global current_state
    print(f"[DFA] Current state: {current_state} | Event: {event}")

    state_def = DFA.get(current_state)
    if not state_def:
        return f"[ERROR] Undefined state: {current_state}"

    next_state = resolve_event_trigger(state_def, event)
    if not next_state:
        return f"[DFA] No transition for event '{event}' in state '{current_state}'"

    print(f"[DFA] Transition: {current_state} → {next_state}")
    current_state = next_state
    execute_state(current_state)
    return f"[DFA] State changed to {current_state}"

def resolve_event_trigger(state_def, event):
    transitions = state_def.get("transitions", {})

    if event in transitions:
        return transitions[event]

    for trigger_key in transitions:
        if trigger_key.endswith(":*"):
            prefix = trigger_key[:-2]
            if event.startswith(prefix + ":"):
                return transitions[trigger_key]

    return None

def execute_state(state_name: str):
    state = DFA.get(state_name)
    if not state:
        print(f"[DFA] Invalid state: {state_name}")
        return

    actions = state.get("on_enter", [])
    print(f"[DFA] Executing {len(actions)} actions for state '{state_name}'")
    for action in actions:
        perform_action(action)

# === Event polling loop ===
def start_dfa_loop():
    from threading import Thread
    from PhysicalDriver import (
        perform_action,
        wait_for_button,
        wait_for_ir_signal,
        init_button_matrix,
        setup_gpio,
        lcd_init,
        send_ir_thread,
        receive_ir_thread,
        button_scan_thread
    )

    # Start background threads
    Thread(target=send_ir_thread, daemon=True).start()
    Thread(target=receive_ir_thread, daemon=True).start()
    Thread(target=button_scan_thread, daemon=True).start()

    execute_state(current_state)  # Initialize first state
    while True:
        # First check buttons
        button = wait_for_button(timeout=0.1)
        if button:
            trigger = f"Button:{button}"
            handle_command(trigger)
            continue

        # Then check IR
        ir_signal = wait_for_ir_signal(timeout=0.1)
        if ir_signal:
            trigger = f"IR_Received:{ir_signal}"
            handle_command(trigger)

        # Add a small sleep to reduce CPU usage
        time.sleep(0.05)

# Optional entry point
if __name__ == "__main__":
    setup_gpio()
    lcd_init()
    init_button_matrix()
    start_dfa_loop()
