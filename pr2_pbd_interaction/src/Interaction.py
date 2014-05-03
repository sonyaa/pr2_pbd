'''Main interaction loop'''
from geometry_msgs.msg import Pose, Point, Quaternion

import roslib
import tf

roslib.load_manifest('pr2_pbd_interaction')

# Generic libraries
import rospy
import time
from visualization_msgs.msg import MarkerArray

# Local stuff
from World import World
from RobotSpeech import RobotSpeech
from Session_old import Session
from Response import Response
from Robot import Robot
from Arm import ArmMode
from pr2_pbd_interaction.msg import ArmState, GripperState
from pr2_pbd_interaction.msg import ActionStep, ArmTarget, Object
from pr2_pbd_interaction.msg import GripperAction, ArmTrajectory
from pr2_pbd_interaction.msg import ExecutionStatus, GuiCommand
from pr2_pbd_interaction.msg import Condition
from pr2_pbd_speech_recognition.msg import Command
from pr2_social_gaze.msg import GazeGoal


class Interaction:
    '''Finite state machine for the human interaction'''

    _is_programming = True
    _is_recording_motion = False
    _arm_trajectory = None
    _trajectory_start_time = None

    def __init__(self):
        self.robot = Robot.get_robot()
        self.world = World()
        self.session = Session(object_list=self.world.get_frame_list(),
                               is_debug=True)
        self._viz_publisher = rospy.Publisher('visualization_marker_array',
                                              MarkerArray)

        rospy.Subscriber('recognized_command', Command, self.speech_command_cb)
        rospy.Subscriber('gui_command', GuiCommand, self.gui_command_cb)

        self._undo_function = None

        self.responses = {
            Command.TEST_MICROPHONE: Response(Interaction.empty_response,
                                [RobotSpeech.TEST_RESPONSE, GazeGoal.NOD]),
            Command.RELAX_RIGHT_ARM: Response(self.relax_arm, 0),
            Command.RELAX_LEFT_ARM: Response(self.relax_arm, 1),
            Command.OPEN_RIGHT_HAND: Response(self.open_hand, 0),
            Command.OPEN_LEFT_HAND: Response(self.open_hand, 1),
            Command.CLOSE_RIGHT_HAND: Response(self.close_hand, 0),
            Command.CLOSE_LEFT_HAND: Response(self.close_hand, 1),
            Command.STOP_EXECUTION: Response(self.stop_execution, None),
            Command.UNDO: Response(self.undo, None),
            Command.DELETE_ALL_STEPS: Response(self.delete_all_steps, None),
            Command.DELETE_LAST_STEP: Response(self.delete_last_step, None),
            Command.FREEZE_RIGHT_ARM: Response(self.freeze_arm, 0),
            Command.FREEZE_LEFT_ARM: Response(self.freeze_arm, 1),
            Command.CREATE_NEW_ACTION: Response(self.create_action, None),
            Command.EXECUTE_ACTION: Response(self.execute_action, None),
            Command.NEXT_ACTION: Response(self.next_action, None),
            Command.PREV_ACTION: Response(self.previous_action, None),
            Command.SAVE_POSE: Response(self.save_step, None),
            Command.RECORD_OBJECT_POSE: Response(
                                            self.record_object_pose, None),
            Command.START_RECORDING_MOTION: Response(
                                            self.start_recording, None),
            Command.STOP_RECORDING_MOTION: Response(self.stop_recording, None),
            Command.EXECUTE_GENERATED_ACTION: Response(self.execute_generated_action, None),
            Command.CALCULATE_POSE_DISTRIBUTION: Response(self.calculate_pose_distribution, None),
            Command.CONTINUE_EXECUTION: Response(self.continue_execution, None)
            }

        rospy.loginfo('Interaction initialized.')

    def open_hand(self, arm_index):
        initial_condition = Condition(self.robot.get_gripper_position(0),
                                          self.robot.get_gripper_position(1))
        '''Opens gripper on the indicated side'''
        if self.robot.set_gripper_state(arm_index, GripperState.OPEN):
            speech_response = Response.open_responses[arm_index]
            if (Interaction._is_programming and self.session.n_actions() > 0):
                self.save_gripper_step(arm_index, GripperState.OPEN, initial_condition)
                speech_response = (speech_response + ' ' +
                                   RobotSpeech.STEP_RECORDED)
            return [speech_response, Response.glance_actions[arm_index]]
        else:
            return [Response.already_open_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def close_hand(self, arm_index):
        '''Closes gripper on the indicated side'''
        initial_condition = Condition(self.robot.get_gripper_position(0),
                                          self.robot.get_gripper_position(1))
        if Robot.set_gripper_state(arm_index, GripperState.CLOSED):
            speech_response = Response.close_responses[arm_index]
            if (Interaction._is_programming and self.session.n_actions() > 0):
                self.save_gripper_step(arm_index, GripperState.CLOSED, initial_condition)
                speech_response = (speech_response + ' ' +
                                   RobotSpeech.STEP_RECORDED)
            return [speech_response, Response.glance_actions[arm_index]]
        else:
            return [Response.already_closed_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def relax_arm(self, arm_index):
        '''Relaxes arm on the indicated side'''
        if self.robot.set_arm_mode(arm_index, ArmMode.RELEASE):
            return [Response.release_responses[arm_index],
                    Response.glance_actions[arm_index]]
        else:
            return [Response.already_released_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def freeze_arm(self, arm_index):
        '''Stiffens arm on the indicated side'''
        if self.robot.set_arm_mode(arm_index, ArmMode.HOLD):
            return [Response.hold_responses[arm_index],
                    Response.glance_actions[arm_index]]
        else:
            return [Response.already_holding_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def edit_action(self, dummy=None):
        '''Goes back to edit mode'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                return [RobotSpeech.ALREADY_EDITING, GazeGoal.SHAKE]
            else:
                Interaction._is_programming = True
                return [RobotSpeech.SWITCH_TO_EDIT_MODE, GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def save_action(self, dummy=None):
        '''Goes out of edit mode'''
        self.session.save_current_action()
        Interaction._is_programming = False
        return [RobotSpeech.ACTION_SAVED + ' ' +
                str(self.session.current_action_index), GazeGoal.NOD]

    def create_action(self, dummy=None):
        '''Creates a new empty action'''
        self.world.clear_all_objects()
        self.session.new_action()
        Interaction._is_programming = True
        return [RobotSpeech.SKILL_CREATED + ' ' +
                str(self.session.current_action_index), GazeGoal.NOD]

    def next_action(self, dummy=None):
        '''Switches to next action'''
        if (self.session.n_actions() > 0):
            if self.session.next_action(self.world.get_frame_list()):
                return [RobotSpeech.SWITCH_SKILL + ' ' +
                        str(self.session.current_action_index), GazeGoal.NOD]
            else:
                return [RobotSpeech.ERROR_NEXT_SKILL + ' ' +
                        str(self.session.current_action_index), GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def previous_action(self, dummy=None):
        '''Switches to previous action'''
        if (self.session.n_actions() > 0):
            if self.session.previous_action(self.world.get_frame_list()):
                return [RobotSpeech.SWITCH_SKILL + ' ' +
                        str(self.session.current_action_index), GazeGoal.NOD]
            else:
                return [RobotSpeech.ERROR_PREV_SKILL + ' ' +
                        str(self.session.current_action_index), GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def delete_last_step(self, dummy=None):
        '''Deletes last step of the current action'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                if self.session.n_frames() > 0:
                    self.session.delete_last_step()
                    self._undo_function = self._resume_last_step
                    return [RobotSpeech.LAST_POSE_DELETED, GazeGoal.NOD]
                else:
                    return [RobotSpeech.SKILL_EMPTY, GazeGoal.SHAKE]
            else:
                return ['Action ' + str(self.session.current_action_index) +
                        RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def delete_all_steps(self, dummy=None):
        '''Deletes all steps in the current action'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                if self.session.n_frames() > 0:
                    self.session.clear_current_action()
                    self._undo_function = self._resume_all_steps
                    return [RobotSpeech.SKILL_CLEARED, GazeGoal.NOD]
                else:
                    return [RobotSpeech.SKILL_EMPTY, None]
            else:
                return ['Action ' + str(self.session.current_action_index) +
                        RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def undo(self, dummy=None):
        '''Undoes the effect of the previous command'''
        if (self._undo_function == None):
            return [RobotSpeech.ERROR_NOTHING_TO_UNDO, GazeGoal.SHAKE]
        else:
            return self._undo_function()

    def _resume_all_steps(self):
        '''Resumes all steps after clearing'''
        self.session.undo_clear()
        return [RobotSpeech.ALL_POSES_RESUMED, GazeGoal.NOD]

    def _resume_last_step(self):
        '''Resumes last step after deleting'''
        self.session.resume_deleted_step()
        return [RobotSpeech.POSE_RESUMED, GazeGoal.NOD]

    def stop_execution(self, dummy=None):
        '''Stops ongoing execution'''
        if (self.robot.is_executing() or self.robot.is_condition_error()):
            self.robot.stop_execution()
            return [RobotSpeech.STOPPING_EXECUTION, GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_NO_EXECUTION, GazeGoal.SHAKE]

    def continue_execution(self, dummy=None):
        '''Continues execution that was interrupted because of condition error'''
        if (self.robot.is_executing() or self.robot.is_condition_error()):
            self.robot.continue_execution()
            return [RobotSpeech.CONTINUING_EXECUTION, GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_NO_CONDITION_ERROR, GazeGoal.SHAKE]

    def save_gripper_step(self, arm_index, gripper_state, initial_condition):
        '''Saves an action step that involves a gripper state change'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                states = self._get_arm_states()
                step = ActionStep()
                step.type = ActionStep.ARM_TARGET
                step.armTarget = ArmTarget(states[0], states[1], 0.2, 0.2)
                actions = [self.robot.get_gripper_state(0),
                           self.robot.get_gripper_state(1)]
                actions[arm_index] = gripper_state
                step.gripperAction = GripperAction(actions[0], actions[1])
                prev_step = self.session.get_current_action().get_last_step()
                step.preCond = initial_condition if prev_step is None else prev_step.postCond
                step.postCond = Condition(self.robot.get_gripper_position(0),
                                          self.robot.get_gripper_position(1))
                step.baseTarget.pose = self._get_base_state()
                self.session.add_step_to_action(step,
                                                self.world.get_frame_list())

    def start_recording(self, dummy=None):
        '''Starts recording continuous motion'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                if (not Interaction._is_recording_motion):
                    Interaction._is_recording_motion = True
                    Interaction._arm_trajectory = ArmTrajectory()
                    Interaction._trajectory_start_time = rospy.Time.now()
                    return [RobotSpeech.STARTED_RECORDING_MOTION,
                            GazeGoal.NOD]
                else:
                    return [RobotSpeech.ALREADY_RECORDING_MOTION,
                            GazeGoal.SHAKE]
            else:
                return ['Action ' + str(self.session.current_action_index) +
                        RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def stop_recording(self, dummy=None):
        '''Stops recording continuous motion'''
        if (Interaction._is_recording_motion):
            Interaction._is_recording_motion = False
            traj_step = ActionStep()
            traj_step.type = ActionStep.ARM_TRAJECTORY

            waited_time = Interaction._arm_trajectory.timing[0]
            for i in range(len(Interaction._arm_trajectory.timing)):
                Interaction._arm_trajectory.timing[i] -= waited_time
                Interaction._arm_trajectory.timing[i] += rospy.Duration(0.1)

            self._fix_trajectory_ref()
            traj_step.arm_trajectory = ArmTrajectory(
                Interaction._arm_trajectory.rArm[:],
                Interaction._arm_trajectory.lArm[:],
                Interaction._arm_trajectory.timing[:],
                Interaction._arm_trajectory.r_ref,
                Interaction._arm_trajectory.l_ref,
                Interaction._arm_trajectory.r_ref_name,
                Interaction._arm_trajectory.l_ref_name)
            traj_step.gripperAction = GripperAction(
                                        self.robot.get_gripper_state(0),
                                        self.robot.get_gripper_state(1))
            #traj_step.baseTarget.pose = self._get_base_state()
            self.session.add_step_to_action(traj_step,
                                        self.world.get_frame_list())
            Interaction._arm_trajectory = None
            Interaction._trajectory_start_time = None
            return [RobotSpeech.STOPPED_RECORDING_MOTION + ' ' +
                    RobotSpeech.STEP_RECORDED, GazeGoal.NOD]
        else:
            return [RobotSpeech.MOTION_NOT_RECORDING, GazeGoal.SHAKE]

    def _fix_trajectory_ref(self):
        '''Makes the reference frame of continuous trajectories uniform'''
        r_ref, r_ref_name = self._find_dominant_ref(
                                        Interaction._arm_trajectory.rArm)
        l_ref, l_ref_name = self._find_dominant_ref(
                                        Interaction._arm_trajectory.lArm)
        for i in range(len(Interaction._arm_trajectory.timing)):
            Interaction._arm_trajectory.rArm[i] = World.convert_ref_frame(
                            Interaction._arm_trajectory.rArm[i],
                            r_ref, r_ref_name)
            Interaction._arm_trajectory.lArm[i] = World.convert_ref_frame(
                            Interaction._arm_trajectory.lArm[i],
                            l_ref, l_ref_name)
        Interaction._arm_trajectory.r_ref = r_ref
        Interaction._arm_trajectory.l_ref = l_ref
        Interaction._arm_trajectory.r_ref_name = r_ref_name
        Interaction._arm_trajectory.l_ref_name = l_ref_name

    def _find_dominant_ref(self, arm_traj):
        '''Finds the most dominant reference frame
        in a continuous trajectory'''
        ref_names = self.world.get_frame_list()
        ref_counts = dict()
        for i in range(len(ref_names)):
            ref_counts[ref_names[i]] = 0
        for i in range(len(arm_traj)):
            if (arm_traj[i].refFrameName in ref_counts.keys()):
                ref_counts[arm_traj[i].refFrameName] += 1
            else:
                rospy.logwarn('Ignoring object with reference frame name '
                    + arm_traj[i].refFrameName
                    + ' because the world does not have this object.')
        dominant_ref = ref_counts.values().index(
                                                max(ref_counts.values()))
        dominant_ref_name = ref_counts.keys()[dominant_ref]
        return World.get_ref_from_name(dominant_ref_name), dominant_ref_name

    def _save_arm_to_trajectory(self):
        '''Saves current arm state into continuous trajectory'''
        if (Interaction._arm_trajectory != None):
            states = self._get_arm_states()
            Interaction._arm_trajectory.rArm.append(states[0])
            Interaction._arm_trajectory.lArm.append(states[1])
            Interaction._arm_trajectory.timing.append(
                        rospy.Time.now() - Interaction._trajectory_start_time)

    def save_step(self, dummy=None):
        '''Saves current arm state as an action step'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                states = self._get_arm_states()
                step = ActionStep()
                step.type = ActionStep.ARM_TARGET
                step.armTarget = ArmTarget(states[0], states[1],
                                           0.2, 0.2)
                step.gripperAction = GripperAction(
                                            self.robot.get_gripper_state(0),
                                            self.robot.get_gripper_state(1))
                prev_step = self.session.get_current_action().get_last_step()
                if prev_step is None:
                    step.preCond = Condition(self.robot.get_gripper_position(0),
                                          self.robot.get_gripper_position(1))
                else:
                    step.preCond = prev_step.postCond
                step.postCond = Condition(self.robot.get_gripper_position(0),
                                          self.robot.get_gripper_position(1))
                rospy.loginfo("post")
                rospy.loginfo(step.postCond)
                step.baseTarget.pose = self.robot.get_base_state()
                self.session.add_step_to_action(step,
                                            self.world.get_frame_list())
                return [RobotSpeech.STEP_RECORDED, GazeGoal.NOD]
            else:
                return ['Action ' + str(self.session.current_action_index) +
                        RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]


    def _get_arm_states(self):
        '''Returns the current arms states in the right format'''
        abs_ee_poses = [Robot.get_ee_state(0),
                      Robot.get_ee_state(1)]
        joint_poses = [Robot.get_joint_state(0),
                      Robot.get_joint_state(1)]

        rel_ee_poses = [None, None]
        states = [None, None]

        nearest_objects = [None, None]
        for arm_index in [0, 1]:
            if (not World.has_objects()):
                # Absolute
                states[arm_index] = ArmState(ArmState.ROBOT_BASE,
                    abs_ee_poses[arm_index], joint_poses[arm_index], Object())
            else:
                nearest_obj = self.world.get_nearest_object(
                                                    abs_ee_poses[arm_index])
                nearest_objects[arm_index] = nearest_obj
                if (nearest_obj == None):
                    states[arm_index] = ArmState(ArmState.ROBOT_BASE,
                                        abs_ee_poses[arm_index],
                                        joint_poses[arm_index], Object())
                else:
                    # Relative
		    #rospy.loginfo('Pose is relative for arm ' + str(arm_index))
                    rel_ee_poses[arm_index] = World.transform(
                                        abs_ee_poses[arm_index],
                                        'base_link', nearest_obj.name)
                    states[arm_index] = ArmState(ArmState.OBJECT,
                                        rel_ee_poses[arm_index],
                                        joint_poses[arm_index], nearest_obj)
        self.world.process_nearest_objects(nearest_objects)
        return states

    def execute_action(self, dummy=None):
        '''Starts the execution of the current action'''
        if (self.session.n_actions() > 0):
            if (self.session.n_frames() > 1):
                self.session.save_current_action()
                action = self.session.get_current_action()

                if (action.is_object_required()):
                    object_pose_result = self.record_object_pose()
                    if object_pose_result[0] == RobotSpeech.START_STATE_RECORDED:
                        self.robot.start_execution(action)
                    else:
                        return [RobotSpeech.OBJECT_NOT_DETECTED,
                                GazeGoal.SHAKE]
                else:
                    self.robot.start_execution(action)

                return [RobotSpeech.START_EXECUTION + ' ' +
                        str(self.session.current_action_index), None]
            else:
                return [RobotSpeech.EXECUTION_ERROR_NOPOSES + ' ' +
                        str(self.session.current_action_index), GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def speech_command_cb(self, command):
        '''Callback for when a speech command is receieved'''
        if command.command in self.responses.keys():
            rospy.loginfo('\033[32m Calling response for command ' +
                          command.command + '\033[0m')
            response = self.responses[command.command]

            if (self.robot.is_condition_error()):
                if command.command == Command.STOP_EXECUTION or command.command == Command.CONTINUE_EXECUTION:
                    response.respond()
                else:
                    rospy.logwarn('Ignoring speech command during condition error: '
                                  + command.command)
            elif (not self.robot.is_executing()):
                if (self._undo_function != None):
                    response.respond()
                    self._undo_function = None
                else:
                    response.respond()
            else:
                if command.command == Command.STOP_EXECUTION or command.command == Command.CONTINUE_EXECUTION:
                    response.respond()
                else:
                    rospy.logwarn('Ignoring speech command during execution: '
                                  + command.command)
        else:
            switch_command = 'SWITCH_TO_ACTION'
            if (switch_command in command.command):
                action_no = command.command[
                                len(switch_command):len(command.command)]
                action_no = int(action_no)
                if (self.session.n_actions() > 0):
                    self.session.switch_to_action(action_no,
                                                  self.world.get_frame_list())
                    response = Response(Interaction.empty_response,
                        [RobotSpeech.SWITCH_SKILL + str(action_no),
                         GazeGoal.NOD])
                else:
                    response = Response(Interaction.empty_response,
                        [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE])
                response.respond()
            else:
                rospy.logwarn('\033[32m This command (' + command.command
                              + ') is unknown. \033[0m')

    def gui_command_cb(self, command):
        '''Callback for when a GUI command is received'''

        if (not self.robot.is_executing() and not self.robot.is_condition_error()):
            if (self.session.n_actions() > 0):
                if (command.command == GuiCommand.SWITCH_TO_ACTION):
                    action_no = command.param
                    self.session.switch_to_action(action_no,
                                                  self.world.get_frame_list())
                    response = Response(Interaction.empty_response,
                        [RobotSpeech.SWITCH_SKILL + str(action_no),
                         GazeGoal.NOD])
                    response.respond()
                elif (command.command == GuiCommand.SELECT_ACTION_STEP):
                    step_no = command.param
                    self.session.select_action_step(step_no)
                    rospy.loginfo('Selected action step ' + str(step_no))
                elif (command.command == GuiCommand.DESELECT_ACTION_STEP):
                    step_no = command.param
                    self.session.deselect_action_step(step_no)
                    rospy.loginfo('Deselected action step ' + str(step_no))
                else:
                    rospy.logwarn('\033[32m This command (' + command.command
                                  + ') is unknown. \033[0m')
            else:
                response = Response(Interaction.empty_response,
                    [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE])
                response.respond()
        else:
            rospy.logwarn('Ignoring GUI command during execution: ' +
                                command.command)

    def update(self):
        '''General update for the main loop'''
        self.robot.update()

        if (self.robot.status != ExecutionStatus.NOT_EXECUTING):
            if (self.robot.status == ExecutionStatus.CONDITION_ERROR):
                Response.say(RobotSpeech.CONDITION_ERROR)
                self.robot.status = ExecutionStatus.EXECUTING
                Response.perform_gaze_action(GazeGoal.SHAKE)
            elif (self.robot.status != ExecutionStatus.EXECUTING):
                self._end_execution()
        if (Interaction._is_recording_motion):
            self._save_arm_to_trajectory()

        is_world_changed = self.world.update()
        if (self.session.n_actions() > 0):
            action = self.session.get_current_action()
            action.update_viz()
            r_target = action.get_requested_targets(0)
            if (r_target != None):
                self.robot.start_move_to_pose(r_target, 0)
                action.reset_targets(0)
            l_target = action.get_requested_targets(1)
            if (l_target != None):
                self.robot.start_move_to_pose(l_target, 1)
                action.reset_targets(1)

            action.delete_requested_steps()

            states = self._get_arm_states()
            action.change_requested_steps(states[0], states[1])

            if (is_world_changed):
                rospy.loginfo('The world has changed.')
                self.session.get_current_action().update_objects(
                                        self.world.get_frame_list())

        time.sleep(0.1)

    def _end_execution(self):
        '''Responses for when the action execution ends'''
        if (self.robot.status == ExecutionStatus.SUCCEEDED):
            Response.say(RobotSpeech.EXECUTION_ENDED)
            Response.perform_gaze_action(GazeGoal.NOD)
        elif (self.robot.status == ExecutionStatus.PREEMPTED):
            Response.say(RobotSpeech.EXECUTION_PREEMPTED)
            Response.perform_gaze_action(GazeGoal.SHAKE)
        else:
            Response.say(RobotSpeech.EXECUTION_ERROR_NOIK)
            Response.perform_gaze_action(GazeGoal.SHAKE)

        self.robot.status = ExecutionStatus.NOT_EXECUTING

    def record_object_pose(self, dummy=None):
        '''Makes the robot look for a table and objects'''
        # Remember current pose.
        #states = self._get_arm_states()
        #armTarget = ArmTarget(states[0], states[1], 0.1, 0.1)
        #arms_status = self.arms.status
        ## Pretend that we're in the execution, so the robot's gaze doesn't follow the arms.
        #self.arms.status = ExecutionStatus.EXECUTING
        #rospy.loginfo("Moving arms out of the way to record object poses.")
        ## Move to predefined pose with arms out of the way.
        #Robot.arms[0].move_to_joints([-1.305, -0.03, -1.55, -1.32, 19.47, -0.17, -5.03], 1)
        #Robot.arms[1].move_to_joints([1.305, 0.03, 1.55, -1.32, -19.47, 0.17, 5.03], 1)
        #time.sleep(2)
        #response = None
        if (self.world.update_object_pose()):
            if (self.session.n_actions() > 0):
                self.session.get_current_action().update_objects(
                                            self.world.get_frame_list())
            response = [RobotSpeech.START_STATE_RECORDED, GazeGoal.NOD]
        else:
            response = [RobotSpeech.OBJECT_NOT_DETECTED, GazeGoal.SHAKE]
        #self.arms.status = arms_status
        ## Return to remembered pose.
        #rospy.loginfo("Moving arms back after recording object poses.")
        #self.arms.move_to_joints(armTarget.rArm, armTarget.lArm)
        return response

    def save_experiment_state(self):
        '''Saves session state'''
        self.session.save_current_action()


    def execute_generated_action(self, dummy=None):
        '''Samples a new action out of the pose distributions'''
        if (self.session.n_actions() > 0):
            if (self.session.n_frames() > 1):
                # Update pose distibutions.
                pose_distribution_result = self.calculate_pose_distribution()
                if pose_distribution_result[0] != RobotSpeech.POSE_DISTRIBUTIONS_CALCULATED:
                    return pose_distribution_result
                self.session.save_current_action()
                self.session.get_current_action().reset_viz()
                # Will require an object if the first action required an object.
                if (self.session.actions[1].is_object_required()):
                    object_pose_result = self.record_object_pose()
                    if object_pose_result[0] != RobotSpeech.START_STATE_RECORDED:
                        return [RobotSpeech.OBJECT_NOT_DETECTED, GazeGoal.SHAKE]
                # Generate an action only after the objects were detected or if the object is not required.
                rospy.loginfo("Generating an action from the pose distributions.")
                action = self.session.action_distribution.get_generated_action(self.world.get_frame_list())
                # Check is all poses are reachable. If not, try to execute one of the existing (original) actions.
                if not Robot.is_action_reachable(action):
                    rospy.loginfo("Generated action has unreachable poses. Will try to execute original demonstration.")
                    for (index, action) in self.session.actions.items():
                        if Robot.is_action_reachable(action):
                            rospy.loginfo("Will execute action " + str(index))
                            self.session.switch_to_action(index, self.world.get_frame_list())
                            break
                    # If no original action can be executed, try to generate an action for n_tries.
                    if not Robot.is_action_reachable(action):
                        n_tries = 10
                        rospy.loginfo("No original demonstration was reachable, "
                                      "will try to generate " + str(n_tries) + " more actions.")
                        for i in range(n_tries):
                            action = self.session.action_distribution.get_generated_action(self.world.get_frame_list())
                            if Robot.is_action_reachable(action):
                                break
                self.robot.start_execution(action)
                return [RobotSpeech.START_EXECUTION, None]
            else:
                return [RobotSpeech.EXECUTION_ERROR_NOPOSES + ' ' +
                        str(self.session.current_action_index), GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def calculate_pose_distribution(self, dummy=None):
        if self.session.n_actions() > 0:
            if self.session.is_number_of_frames_consistent():
                self.session.calculate_action_distribution()
                return [RobotSpeech.POSE_DISTRIBUTIONS_CALCULATED, GazeGoal.NOD]
            else:
                return [RobotSpeech.ERROR_WRONG_NUMBER_OF_POSES, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    @staticmethod
    def empty_response(responses):
        '''Default response to speech commands'''
        return responses
