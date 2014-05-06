#!/usr/bin/env python
from Exceptions import NoObjectError
from World import World
from step_types.Step import Step


class ObjectDetectionStep(Step):
    """ Step that looks for objects on the table.
    """

    def execute(self):
        # call object detection
        world = World.get_world()
        if not world.update_object_pose():
            raise NoObjectError()