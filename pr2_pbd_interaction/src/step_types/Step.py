#!/usr/bin/env python
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import rospy
from visualization_msgs.msg import MarkerArray
from condition_types.PreviousStepCondition import PreviousStepCondition
from condition_types.PreviousStepNotFailedCondition import PreviousStepNotFailedCondition
from condition_types.PreviousStepNotSkippedCondition import PreviousStepNotSkippedCondition
from pr2_pbd_interaction.msg import Strategy


class Step:
    """ General step of an action
    """

    marker_publisher = None
    interactive_marker_server = None

    def __init__(self, *args, **kwargs):
        self.strategy = Strategy.FAIL_FAST
        # If self.is_while, execute step in a loop until a condition fails. Else execute step once.
        self.is_while = False
        self.ignore_conditions = False
        self.conditions = [PreviousStepNotFailedCondition(), PreviousStepNotSkippedCondition()]
        self.condition_order = range(len(self.conditions))
        self.prev_step_status = None
        self.execution_status = None
        if Step.interactive_marker_server is None:
            im_server = InteractiveMarkerServer('programmed_actions')
            Step.interactive_marker_server = im_server
        if Step.marker_publisher is None:
            Step.marker_publisher = rospy.Publisher(
                'visualization_marker_array', MarkerArray)

    def set_is_while(self, is_while):
        self.is_while = is_while

    def set_condition_strategy(self, condition_index, strategy_index):
        if condition_index < len(self.conditions):
            self.conditions[condition_index].set_strategy_index(strategy_index)
            rospy.loginfo("Changing strategy for condition " + str(condition_index) + " to " + str(strategy_index))
        else:
            rospy.logwarn("Invalid condition index: " + str(condition_index))

    def add_condition(self, condition):
        self.conditions.append(condition)

    def remove_condition(self, index):
        self.conditions.pop(index)

    def set_condition_order(self, order):
        self.condition_order = order

    def set_previous_step_status(self, status):
        self.prev_step_status = status
        for condition in self.conditions:
            if isinstance(condition, PreviousStepCondition):
                condition.set_prev_step_status(status)

    def get_execution_status(self):
        return self.execution_status

    def execute(self, action_data):
        pass

    def initialize_viz(self):
        self.reset_viz()

    def update_viz(self):
        pass

    def reset_viz(self):
        self.interactive_marker_server.clear()
        self.interactive_marker_server.applyChanges()