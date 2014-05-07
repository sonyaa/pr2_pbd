class ExecutionError(Exception):
    """Base class for exceptions that occur during action execution."""

    def __init__(self, msg="An error occurred during execution"):
        self.msg = msg

    def __str__(self):
        return self.msg

class ConditionError(ExecutionError):
    """ Exception raised when the condition is not satisfied and the strategy is to fail-fast.
    """

    def __init__(self, msg="A condition was not satisfied"):
        self.msg = msg

    def __str__(self):
        return self.msg

class NoObjectError(ExecutionError):
    """ Exception raised when no object is detected when one is needed.
    """

    def __init__(self, msg="The required objects were not found"):
        self.msg = msg

    def __str__(self):
        return self.msg

class UnreachablePoseError(ExecutionError):
    """ Exception raised when there are unreachable poses in the action.
    """

    def __init__(self, msg="Unreachable poses in the action"):
        self.msg = msg

    def __str__(self):
        return self.msg

class BaseObstructedError(ExecutionError):
    """ Exception raised when base could not reach target.
    """

    def __init__(self, msg="Base couldn't reach target"):
        self.msg = msg

    def __str__(self):
        return self.msg

class ArmObstructedError(ExecutionError):
    """ Exception raised when arm could not reach target.
    """

    def __init__(self, msg="Arm couldn't reach target"):
        self.msg = msg

    def __str__(self):
        return self.msg

class StoppedByUserError(ExecutionError):
    """ Exception raised when execution is stopped by user.
    """

    def __init__(self, msg="Execution stopped by user"):
        self.msg = msg

    def __str__(self):
        return self.msg