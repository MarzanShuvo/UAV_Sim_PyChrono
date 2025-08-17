# acsl_pychrono/control/__init__.py

from .PID import PID, PIDGains, PIDLogger
from .MRAC import MRAC, MRACGains, MRACLogger
from .BackStepping import BackStepping, BackSteppingGains, BackSteppingLogger

# Make lookup case-insensitive by normalizing to UPPER
controller_classes = {
    'PID':          (PIDGains,          PID,          PIDLogger),
    'MRAC':         (MRACGains,         MRAC,         MRACLogger),
    'BACKSTEPPING': (BackSteppingGains, BackStepping, BackSteppingLogger),
}

def instantiateController(controller_type: str, ode_input, flight_params, timestep):
    ct = (controller_type or "").strip().upper()
    if ct not in controller_classes:
        allowed = ", ".join(controller_classes.keys())
        raise ValueError(f"Unknown controller type: {controller_type}. Allowed: {allowed}")

    GainsClass, ControllerClass, LoggerClass = controller_classes[ct]
    gains = GainsClass(flight_params)              # mirrors PID pattern
    controller = ControllerClass(gains, ode_input, flight_params, timestep)
    logger = LoggerClass(gains)                    # mirrors PID pattern
    return gains, controller, logger
