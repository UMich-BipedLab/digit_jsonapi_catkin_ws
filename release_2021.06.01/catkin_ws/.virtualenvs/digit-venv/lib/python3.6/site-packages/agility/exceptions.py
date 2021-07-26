class JsonApiError(Exception):
    """Raised when an error is received in response to a sent message."""
    pass

class ActionFailed(Exception):
    """Raised when an action fails."""
    pass

class ActionCanceled(Exception):
    """Raised when an action stops running unexpectedly."""
    pass

class PrivilegeNotObtained(Exception):
    """Raised when a requested privilege is not given to the client."""
    pass

class ConnectionClosedError(Exception):
    """Raised when trying to send a message with a closed JsonApi object."""
    pass

class SimulatorTerminated(Exception):
    """Raised when a simulator instance terminates unexpectedly."""
    pass
