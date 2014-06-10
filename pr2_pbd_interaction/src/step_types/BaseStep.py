#!/usr/bin/env python
import rospy
from BaseStepMarker import BaseStepMarker
from Exceptions import BaseObstructedError, ConditionError, StoppedByUserError
from pr2_pbd_interaction.msg import ExecutionStatus, Strategy
from step_types.Step import Step


class BaseStep(Step):
    ''' Step that moves the base.
    '''

    def __init__(self, *args, **kwargs):  #(self, end_pose):
        Step.__init__(self, *args, **kwargs)
        self.end_pose = args[0]
        self.marker = BaseStepMarker(self, self.marker_click_cb, self.interactive_marker_server)


    def execute(self):
        from Robot import Robot
        robot = Robot.get_robot()
        # If self.is_while, execute everything in a loop until a condition fails. Else execute everything once.
        while True:
            if not self.ignore_conditions:
                for condition in self.conditions:
                    if not condition.check():
                        rospy.logwarn("Condition failed when executing base step.")
                        if self.is_while:
                            return
                        if self.strategy == Strategy.FAIL_FAST:
                            rospy.loginfo("Strategy is to fail-fast, stopping.")
                            robot.status = ExecutionStatus.CONDITION_FAILED
                            raise ConditionError()
                        elif self.strategy == Strategy.CONTINUE:
                            rospy.loginfo("Strategy is to continue, skipping this step.")
                            return
                        else:
                            rospy.logwarn("Unknown strategy " + str(self.strategy))
            else:
                rospy.loginfo('Ignoring conditions for base step')
            if robot.preempt:
                # robot.preempt = False
                robot.status = ExecutionStatus.PREEMPTED
                rospy.logerr('Execution of base step failed, execution preempted by user.')
                raise StoppedByUserError()
            # send a request to Robot to move to end_pose
            if not robot.move_base(self.end_pose):
                if robot.preempt:
                    robot.status = ExecutionStatus.PREEMPTED
                    rospy.logerr('Execution of base step failed, execution preempted by user.')
                    raise StoppedByUserError()
                robot.status = ExecutionStatus.OBSTRUCTED
                raise BaseObstructedError()
            if not self.is_while:
                return

    def initialize_viz(self):
        """Initialize visualization"""
        self.reset_viz()
        self.marker = BaseStepMarker(self, self.marker_click_cb, self.interactive_marker_server)
        self.update_viz()

    def update_viz(self):
        self.marker.update_viz()

    def reset_viz(self):
        """Removes all visualization from RViz"""
        self.marker.destroy()
        self.marker = None

    def marker_click_cb(self, is_selected):
        """Callback for when the marker is clicked."""
        self.marker.is_control_visible = is_selected
        self.marker.update_viz()

    def change_requested_steps(self, base_pose):
        """Change a base step to the current location
        if requested through the interactive marker menu"""
        if self.marker.is_edited:
            self.marker.set_target_pose(base_pose)

    def get_requested_targets(self):
        """Get base step that might have been requested from
        the interactive marker menus"""
        pose = None
        if self.marker.is_requested:
            pose = self.marker.get_target_pose()
        return pose

    def reset_targets(self):
        """Resets requests after reaching a previous target"""
        self.marker.pose_reached()