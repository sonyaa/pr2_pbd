#!/usr/bin/env python
from Exceptions import NoObjectError
from World import World
from step_types import Step


class ObjectDetectionStep(Step):
    ''' Step that looks for objects on the table.
    '''

    def execute(self):
        # call object detection
        world = World.get_world()
        if (world.update_object_pose()):
            if (session.n_actions() > 0):
                session.get_current_action().update_objects(
                                            self.world.get_frame_list())
            #response = [RobotSpeech.START_STATE_RECORDED, GazeGoal.NOD]
        else:
            raise NoObjectError()
            #response = [RobotSpeech.OBJECT_NOT_DETECTED, GazeGoal.SHAKE]