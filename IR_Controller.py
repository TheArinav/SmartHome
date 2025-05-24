# IR_Controller.py

from PhysicalDriver import perform_action  # This will execute hardware actions

# Supported DFA triggers:
# - IR_Received:<signal>        e.g., IR_Received:0x1FE48B7
# - Button:0–9, *, #            e.g., Button:1
# - Remote:<command-string>     e.g., Remote:PowerOff

# Example DFA structure (replace with yours or load dynamically)
DFA = {
    "idle": {
        "on_enter": ["display:Idle", "led:green"],
        "transitions": {
            "Button:1": "turn_on_tv",
            "Remote:Power": "turn_on_ac",
            "IR_Received:0x1FE48B7": "handle_tv_remote"
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
        "on_enter": ["display:Remote Control", "led:cyan"],
        "transitions": {
            "Button:0": "idle"
        }
    }
}

# Internal system state
current_state = "idle"

def handle_command(event: str) -> str:
    """Process an input event and perform a DFA state transition if defined."""
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
    """Matches an event to the correct transition based on exact or prefix match."""
    transitions = state_def.get("transitions", {})

    # Exact match first
    if event in transitions:
        return transitions[event]

    # Allow prefix wildcard matching (e.g., Remote:* or IR_Received:*)
    for trigger_key in transitions:
        if trigger_key.endswith(":*"):
            prefix = trigger_key[:-2]
            if event.startswith(prefix + ":"):
                return transitions[trigger_key]

    return None

def execute_state(state_name: str):
    """Executes all on_enter actions for a given state."""
    state = DFA.get(state_name)
    if not state:
        print(f"[DFA] Invalid state: {state_name}")
        return

    actions = state.get("on_enter", [])
    print(f"[DFA] Executing {len(actions)} actions for state '{state_name}'")
    for action in actions:
        perform_action(action)
