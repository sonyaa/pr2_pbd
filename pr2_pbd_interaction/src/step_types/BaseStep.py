#!/usr/bin/env python
from geometry_msgs.msg import Point
import rospy
import yaml
from BaseStepMarker import BaseStepMarker
from Exceptions import BaseObstructedError, ConditionError, StoppedByUserError
from pr2_pbd_interaction.msg import ExecutionStatus, Strategy
from pr2_pbd_interaction.msg._StepExecutionStatus import StepExecutionStatus
from step_types.Step import Step


def base_step_constructor(loader, node):
    fields = loader.construct_mapping(node, deep=True)
    b_step = BaseStep(fields['end_pose'])
    b_step.strategy = fields['strategy']
    b_step.is_while = fields['is_while']
    b_step.ignore_conditions = fields['ignore_conditions']
    b_step.conditions = fields['conditions']
    b_step.head_position = fields.get('head_position', Point(1,0,1))
    return b_step


yaml.add_constructor(u'!BaseStep', base_step_constructor)


class BaseStep(Step):
    ''' Step that moves the base.
    '''

    def __init__(self, *args, **kwargs):  #(self, end_pose):
        Step.__init__(self, *args, **kwargs)
        self.end_pose = args[0]
        self.marker = BaseStepMarker(self, self.marker_click_cb, self.interactive_marker_server)

    def execute(self, action_data):
        from Robot import Robot
        robot = Robot.get_robot()
        # If self.is_while, execute everything in a loop until a condition fails. Else execute everything once.
        while True:
            if not self.ignore_conditions:
                for condition in [self.conditions[i] for i in self.condition_order]:
                    if not condition.check():
                        rospy.logwarn("Condition failed when executing base step.")
                        if self.is_while:
                            #TODO
                            return
                        strategy = condition.available_strategies[condition.current_strategy_index]
                        if strategy == Strategy.FAIL_FAST:
                            rospy.loginfo("Strategy is to fail-fast, stopping.")
                            robot.status = ExecutionStatus.CONDITION_FAILED
                            raise ConditionError()
                        elif strategy == Strategy.SKIP_STEP:
                            rospy.loginfo("Strategy is to skip step, skipping.")
                            self.execution_status = StepExecutionStatus.SKIPPED
                            return
                        elif strategy == Strategy.CONTINUE:
                            rospy.loginfo("Strategy is to continue, ignoring condition failure.")
                        elif strategy == Strategy.GO_TO_PREVIOUS_STEP:
                            rospy.loginfo("Strategy is to go to previous step.")
                            action_data.go_back = True
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

                rospy.logwarn('Base failed to reach target')
                self.execution_status = StepExecutionStatus.FAILED
                return

            self.execution_status = StepExecutionStatus.SUCCEEDED

            if not self.is_while:
                return

    def initialize_viz(self):
        """Initialize visualization"""
        self.reset_viz()
        self.marker = BaseStepMarker(self, self.marker_click_cb, self.interactive_marker_server)
        self.marker.update_menu()
        self.marker.update_viz()

    def update_viz(self):
        if self.marker is not None:
            self.marker.update_viz()

    def reset_viz(self):
        """Removes all visualization from RViz"""
        if self.marker is not None:
            self.marker.destroy()
        self.marker = None

    def marker_click_cb(self, is_selected):
        """Callback for when the marker is clicked."""
        self.marker.is_control_visible = is_selected
        self.marker.update_viz()

    def change_requested_steps(self, base_pose):
        """Change a base step to the current location
        if requested through the interactive marker menu"""
        if self.marker is not None and self.marker.is_edited:
            self.marker.set_target_pose(base_pose)

    def get_requested_targets(self):
        """Get base step that might have been requested from
        the interactive marker menus"""
        pose = None
        if self.marker is not None and self.marker.is_requested:
            pose = self.marker.get_target_pose()
        return pose

    def reset_targets(self):
        """Resets requests after reaching a previous target"""
        self.marker.pose_reached()


def base_step_representer(dumper, data):
    return dumper.represent_mapping(u'!BaseStep', {'strategy': data.strategy,
                                                   'is_while': data.is_while,
                                                   'ignore_conditions': data.ignore_conditions,
                                                   'conditions': data.conditions,
                                                   'end_pose': data.end_pose,
                                                   'head_position': data.head_position})


yaml.add_representer(BaseStep, base_step_representer)