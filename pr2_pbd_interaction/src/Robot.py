#!/usr/bin/env python

from actionlib.simple_action_client import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
import roslib
import tf

roslib.load_manifest('pr2_pbd_interaction')
import rospy

import time
import threading
from step_types.ArmStep import ArmStep
from pr2_pbd_interaction.msg import ArmState, GripperState
from pr2_pbd_interaction.msg import Side
from pr2_pbd_interaction.msg import ExecutionStatus
from pr2_social_gaze.msg import GazeGoal
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion, Twist, Vector3
from Response import Response
from World import World
from Arm import Arm, ArmMode
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseGoal
from pr2_common_action_msgs.msg import TuckArmsAction, TuckArmsGoal
from pr2_pbd_interaction.msg import ArmTarget, Object


class Robot:
    '''
    Controller for the robot: moves the arms and base.
    '''
    arms = []
    robot = None
    r_tucked_with_object_pose = [-0.0175476818422744, 1.1720448201611564, -1.3268105758514066, -1.288722079574422, -31.28968470078213, -2.0089650532319836, -5.841424529413016]

    #NAVIGATE_R_POS = [-0.3594077470836499, 0.971353000916152, -1.9647276598906076, -1.431900313132731, -1.1839177367219755, -0.09817772642188527, -1.622044624784374]
    l_tucked_with_object_pose = [0.058297695494463064, 1.1720448201611564, 1.3268105758514066, -1.288722079574422, 15.589646214067146, -1.0399744971119742, -33.45882012016671]

    #l_tucked_with_object_pose = [0.04014114629328325, 1.0329086176696876, 1.5482390098896939, -1.538017244576857, -6.7181573150253024, -0.09421239912308044, 87.79342441711326]
    #r_tucked_with_object_pose = [0.054793713231851005, 1.293828847790333, -1.5800364314292505, -1.5937539684595152, 0.0480516740641113, -0.09404437901242257, 15.646428218144145]

    r_tucked_with_two_objects = [-0.2827191260284381, 0.8905648493513978, -1.9251200177772456, -2.1230356892776925, 8.328630782689046, -1.7644549538909033, 6.006939952238806]
    l_tucked_with_two_objects = [-0.014577221162327403, 1.0475435393667585, 1.9520124626890674, -1.2932099716273115, -14.586055305344937, -1.6310018431908655, 12.279584063171104]


    def __init__(self):
        r_arm = Arm(Side.RIGHT)
        l_arm = Arm(Side.LEFT)
        Robot.arms = [r_arm, l_arm]
        self.attended_arm = -1
        self.action = None
        self.preempt = False
        self.is_continue_execution = False
        self.z_offset = 0

        rospy.loginfo('Arms have been initialized.')

        Robot.arms[0].set_mode(ArmMode.HOLD)
        Robot.arms[1].set_mode(ArmMode.HOLD)
        Robot.arms[0].check_gripper_state()
        Robot.arms[1].check_gripper_state()
        Robot.arms[Side.RIGHT].close_gripper()
        Robot.arms[Side.LEFT].close_gripper()
        self.status = ExecutionStatus.NOT_EXECUTING

        self.nav_action_client = SimpleActionClient(
            'move_base', MoveBaseAction)
        self.nav_action_client.wait_for_server()
        rospy.loginfo('Got response from move base action server.')

        self.tuck_arms_client = SimpleActionClient('tuck_arms', TuckArmsAction)
        self.tuck_arms_client.wait_for_server()
        rospy.loginfo('Got response from tuck arms action server.')

    @staticmethod
    def get_robot():
        if Robot.robot is None:
            Robot.robot = Robot()
        return Robot.robot

    @staticmethod
    def set_arm_mode(arm_index, mode):
        '''Set arm to stiff or relaxed'''
        if (mode == Robot.arms[arm_index].arm_mode):
            # Already in that mode
            return False
        else:
            Robot.arms[arm_index].set_mode(mode)
            return True

    @staticmethod
    def set_gripper_state(arm_index, gripper_state):
        '''Set gripper to open or closed'''
        if (gripper_state == Robot.get_gripper_state(arm_index)):
            # Already in that mode
            return False
        else:
            if (gripper_state == GripperState.OPEN):
                Robot.arms[arm_index].open_gripper()
            else:
                Robot.arms[arm_index].close_gripper()
        return True

    def is_executing(self):
        '''Whether or not there is an ongoing execution'''
        return (self.status == ExecutionStatus.EXECUTING)

    def start_execution(self, action, z_offset=0):
        ''' Starts execution of an action'''
        # This will take long, create a thread
        self.preempt = False
        self.z_offset = z_offset
        self.action = action
        thread = threading.Thread(group=None, target=self.execute_action,
                                  name='skill_execution_thread')
        thread.start()

    def execute_action(self):
        self.status = ExecutionStatus.EXECUTING
        rospy.loginfo("Starting execution of action " + self.action.get_name())
        try:
            self.action.execute()
        except Exception as e:
            rospy.logerr("Execution of an action failed: " + str(e))
        else:
            self.status = ExecutionStatus.SUCCEEDED

    def continue_execution(self):
        self.is_continue_execution = True

    def stop_execution(self):
        '''Preempts an ongoing execution'''
        self.preempt = True

    def solve_ik_for_manipulation_step(self, manipulation_step):
        '''Computes joint positions for all end-effector poses
        in an manipulation_step'''

        # Go over steps of the manipulation_step
        for i in range(manipulation_step.n_steps()):
            # For each step check step type
            # If arm target action
            if (manipulation_step.arm_steps[i].type == ArmStep.ARM_TARGET):
                # Find frames that are relative and convert to absolute

                r_arm, has_solution_r = Robot.solve_ik_for_arm(0,
                                                               manipulation_step.arm_steps[i].armTarget.rArm,
                                                               self.z_offset)
                l_arm, has_solution_l = Robot.solve_ik_for_arm(1,
                                                               manipulation_step.arm_steps[i].armTarget.lArm,
                                                               self.z_offset)

                manipulation_step.arm_steps[i].armTarget.rArm = r_arm
                manipulation_step.arm_steps[i].armTarget.lArm = l_arm
                if (not has_solution_r) or (not has_solution_l):
                    return False

            if (manipulation_step.arm_steps[i].type == ArmStep.ARM_TRAJECTORY):
                n_frames = len(manipulation_step.arm_steps[i].armTrajectory.timing)
                for j in range(n_frames):
                    r_arm, has_solution_r = Robot.solve_ik_for_arm(0,
                                                                   manipulation_step.arm_steps[
                                                                       i].armTrajectory.r_arm[j],
                                                                   self.z_offset)
                    l_arm, has_solution_l = Robot.solve_ik_for_arm(1,
                                                                   manipulation_step.arm_steps[
                                                                       i].armTrajectory.l_arm[j],
                                                                   self.z_offset)
                    manipulation_step.arm_steps[i].armTrajectory.r_arm[j] = r_arm
                    manipulation_step.arm_steps[i].armTrajectory.l_arm[j] = l_arm
                    if (not has_solution_r) or (not has_solution_l):
                        return False
        return True

    @staticmethod
    def solve_ik_for_arm(arm_index, arm_state, z_offset=0):
        '''Finds an  IK solution for a particular arm pose'''
        # We need to find IK only if the frame is relative to an object
        if (arm_state.refFrame == ArmState.OBJECT):
            #rospy.loginfo('solve_ik_for_arm: Arm ' + str(arm_index) + ' is relative')
            solution = ArmState()
            target_pose = World.transform(arm_state.ee_pose,
                                          arm_state.refFrameObject.name, 'base_link')

            target_pose.position.z = target_pose.position.z + z_offset

            target_joints = Robot.arms[arm_index].get_ik_for_ee(target_pose,
                                                                arm_state.joint_pose)
            if (target_joints == None):
                rospy.logerr('No IK for relative end-effector pose.')
                return solution, False
            else:
                solution.refFrame = ArmState.ROBOT_BASE
                solution.ee_pose = Pose(target_pose.position,
                                        target_pose.orientation)
                solution.joint_pose = target_joints
                return solution, True
        elif (arm_state.refFrame == ArmState.ROBOT_BASE):
            #rospy.loginfo('solve_ik_for_arm: Arm ' + str(arm_index) + ' is absolute')
            pos = arm_state.ee_pose.position
            target_position = Point(pos.x, pos.y, pos.z + z_offset)
            target_pose = Pose(target_position, arm_state.ee_pose.orientation)
            target_joints = Robot.arms[arm_index].get_ik_for_ee(target_pose,
                                                                arm_state.joint_pose)
            if (target_joints == None):
                rospy.logerr('No IK for absolute end-effector pose.')
                return arm_state, False
            else:
                solution = ArmState()
                solution.refFrame = ArmState.ROBOT_BASE
                solution.ee_pose = Pose(arm_state.ee_pose.position,
                                        arm_state.ee_pose.orientation)
                solution.joint_pose = target_joints
                return solution, True
        else:
            return arm_state, True

    def start_move_arm_to_pose(self, arm_state, arm_index):
        '''Creates a thread for moving the arm to a target pose'''
        self.preempt = False
        thread = threading.Thread(group=None, target=self.move_arm_to_pose,
                                  args=(arm_state, arm_index,),
                                  name='move_to_arm_state_thread')
        thread.start()

    def start_move_base_to_pose(self, base_pose):
        '''Creates a thread for moving the base to a target pose'''
        self.preempt = False
        thread = threading.Thread(group=None, target=self.move_base,
                                  args=(base_pose,),
                                  name='move_to_base_pose_thread')
        thread.start()

    def move_arm_to_pose(self, arm_state, arm_index):
        '''The thread function that makes the arm move to
        a target end-effector pose'''
        rospy.loginfo('Started thread to move arm ' + str(arm_index))
        self.status = ExecutionStatus.EXECUTING
        solution, has_solution = Robot.solve_ik_for_arm(arm_index, arm_state)

        if (has_solution):
            if (arm_index == 0):
                is_successful = self.move_to_joints(solution, None)
            else:
                is_successful = self.move_to_joints(None, solution)

            if (is_successful):
                self.status = ExecutionStatus.SUCCEEDED
            else:
                self.status = ExecutionStatus.OBSTRUCTED
        else:
            self.status = ExecutionStatus.NO_IK

    def get_base_state(self):
        try:
            ref_frame = "/map"
            time = World.tf_listener.getLatestCommonTime(ref_frame,
                                                         "/base_link")
            (position, orientation) = World.tf_listener.lookupTransform(
                ref_frame, "/base_link", time)
            base_pose = Pose()
            base_pose.position = Point(position[0], position[1], position[2])
            base_pose.orientation = Quaternion(orientation[0], orientation[1],
                                               orientation[2], orientation[3])
            #rospy.loginfo('Current base pose:')
            #rospy.loginfo(base_pose)
            return base_pose
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logwarn('Something wrong with transform request for base state.')
            return None

    def _get_absolute_arm_states(self):
        abs_ee_poses = [Robot.get_ee_state(0),
                        Robot.get_ee_state(1)]
        joint_poses = [Robot.get_joint_state(0),
                       Robot.get_joint_state(1)]
        states = [None, None]
        for arm_index in [0, 1]:
            states[arm_index] = ArmState(ArmState.ROBOT_BASE,
                                         abs_ee_poses[arm_index], joint_poses[arm_index], Object())
        return states

    @staticmethod
    def is_hand_holding_object(arm_index):
        position = Robot.get_gripper_position(arm_index)
        closed = 0.002
        open = 0.078
        if closed < position < open:
            return True
        return False

    def tuck_arms_with_objects(self):
        right_has_object = Robot.is_hand_holding_object(0)
        left_has_object = Robot.is_hand_holding_object(1)
        if not right_has_object and not left_has_object:
            rospy.loginfo("Hands are not holding objects, will tuck arms completely")
            goal = TuckArmsGoal()
            goal.tuck_left = True
            goal.tuck_right = False
            self.tuck_arms_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
            goal.tuck_right = True
            self.tuck_arms_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
        elif right_has_object and not left_has_object:
            rospy.loginfo("Left hand has no object, right hand has object. Tucking arms accordingly.")
            goal = TuckArmsGoal()
            goal.tuck_left = True
            goal.tuck_right = False
            self.tuck_arms_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
            Robot.arms[0].move_to_joints(Robot.r_tucked_with_object_pose, 1)
            time.sleep(2)
        elif left_has_object and not right_has_object:
            rospy.loginfo("Right hand has no object, left hand has object. Tucking arms accordingly.")
            goal = TuckArmsGoal()
            goal.tuck_left = False
            goal.tuck_right = True
            self.tuck_arms_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
            Robot.arms[1].move_to_joints(Robot.l_tucked_with_object_pose, 1)
            time.sleep(2)
        else:
            rospy.loginfo("Both hands have objects. Tucking arms accordingly.")
            goal = TuckArmsGoal()
            goal.tuck_left = False
            goal.tuck_right = False
            self.tuck_arms_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
            Robot.arms[1].move_to_joints(Robot.l_tucked_with_two_objects, 1)
            time.sleep(2)
            Robot.arms[0].move_to_joints(Robot.r_tucked_with_two_objects, 1)
            time.sleep(2)


    def move_base(self, base_pose):
        '''Moves the base to the desired position'''
        # Setup the goal
        #nav_goal = MoveBaseActionGoal()
        #nav_goal.header.stamp = (rospy.Time.now() +
        #                                     rospy.Duration(0.1))
        #nav_goal.goal_id.stamp = nav_goal.header.stamp
        #nav_goal.goal_id.id = 1
        current_pose = self.get_base_state()
        #rospy.loginfo('Current base pose:')
        #rospy.loginfo(current_pose)
        #rospy.loginfo('Sending base to pose:')
        #rospy.loginfo(base_pose)
        if World.pose_distance(current_pose, base_pose) < 0.2:
            rospy.loginfo("Don't need to move the base, we're already there.")
            return True

        #Remember arm states and tuck arms
        states = self._get_absolute_arm_states()
        armTarget = ArmTarget(states[0], states[1], 0.1, 0.1)
        arms_status = self.status
        # Pretend that we're in the execution, so the robot's gaze doesn't follow the arms.
        self.status = ExecutionStatus.EXECUTING
        rospy.loginfo("Tucking arms for navigation.")
        self.tuck_arms_with_objects()
        self.spin_base(1)

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "/map"
        pose_stamped.pose = base_pose
        #nav_goal.goal = MoveBaseGoal()
        #nav_goal.goal.target_pose = pose_stamped
        #rospy.loginfo(nav_goal)
        nav_goal = MoveBaseGoal()
        nav_goal.target_pose = pose_stamped
        # Client sends the goal to the Server
        self.nav_action_client.send_goal(nav_goal)
        elapsed_time = 0
        while (self.nav_action_client.get_state() == GoalStatus.ACTIVE
               or self.nav_action_client.get_state() == GoalStatus.PENDING):
            time.sleep(0.01)
            elapsed_time += 0.01
            if elapsed_time > 60:
                rospy.loginfo('Timeout waiting for base navigation to finish')
                self.nav_action_client.cancel_goal()
                break
            if self.preempt:
                self.nav_action_client.cancel_goal()
                rospy.logwarn('Execution stopped by user, cannot move base to pose')
                return False
        rospy.loginfo('Done with base navigation.')

        # Untuck arms and move to where they were.
        rospy.loginfo("Untucking arms and moving them back after navigation.")
        goal = TuckArmsGoal()
        goal.tuck_left = False
        goal.tuck_right = False
        self.tuck_arms_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
        # Return to remembered pose.
        self.status = arms_status
        self.move_to_joints(armTarget.rArm, armTarget.lArm)

        # Verify that base succeeded
        if (self.nav_action_client.get_state() != GoalStatus.SUCCEEDED):
            rospy.logwarn('Aborting because base failed to move to pose.')
            return False
        else:
            return True

    def spin_base(self, rotate_count):
        """ Spin 360 * rotate_count degrees clockwise """
        rospy.loginfo("Orienting...")
        if not rotate_count:
            rotate_count = 2
        topic_name = '/base_controller/command'
        base_publisher = rospy.Publisher(topic_name, Twist)

        twist_msg = Twist()
        twist_msg.linear = Vector3(0.0, 0.0, 0.0)
        twist_msg.angular = Vector3(0.0, 0.0, 0.5)
        start_time = rospy.get_rostime()
        while rospy.get_rostime() < start_time + rospy.Duration(15.0 * rotate_count):
            if self.preempt:
                rospy.logwarn('Execution stopped by user, stopping orientation.')
                return
            base_publisher.publish(twist_msg)


    @staticmethod
    def _get_time_to_pose(pose, arm_index):
        ''' Returns the time to get to an arm pose'''
        if (pose == None):
            rospy.logwarn('Arm ' + str(arm_index) + 'will not move.')
            return None
        else:
            time_to_pose = Robot._get_time_bw_poses(
                Robot.arms[arm_index].get_ee_state(),
                pose.ee_pose)
            rospy.loginfo('Duration until next frame for arm ' +
                          str(arm_index) + ': ' + str(time_to_pose))
            return time_to_pose

    def move_to_joints(self, r_arm, l_arm):
        '''Makes the arms move to indicated joint poses'''
        time_to_r_pose = Robot._get_time_to_pose(r_arm, 0)
        time_to_l_pose = Robot._get_time_to_pose(l_arm, 1)

        #  If both arms are moving adjust velocities and find most moving arm
        is_r_moving = (time_to_r_pose != None)
        is_l_moving = (time_to_l_pose != None)
        if (not is_r_moving):
            Response.look_at_point(l_arm.ee_pose.position)
        elif (not is_l_moving):
            Response.look_at_point(r_arm.ee_pose.position)
        else:
            if (time_to_r_pose > time_to_l_pose):
                time_to_l_pose = time_to_r_pose
                Response.look_at_point(r_arm.ee_pose.position)
            else:
                time_to_r_pose = time_to_l_pose
                Response.look_at_point(l_arm.ee_pose.position)

        #  Move arms to target
        if (is_r_moving):
            Robot.arms[0].move_to_joints(r_arm.joint_pose, time_to_r_pose)
        if (is_l_moving):
            Robot.arms[1].move_to_joints(l_arm.joint_pose, time_to_l_pose)

        # Wait until both arms complete the trajectory
        while ((Robot.arms[0].is_executing() or
                    Robot.arms[1].is_executing()) and not self.preempt):
            time.sleep(0.01)
        rospy.loginfo('Arms reached target.')

        # Verify that both arms succeeded
        if ((not Robot.arms[0].is_successful() and is_r_moving) or
                (not Robot.arms[1].is_successful() and is_l_moving)):
            rospy.logwarn('Aborting because arms failed to move to pose.')
            return False
        else:
            return True

    @staticmethod
    def _get_most_moving_arm():
        '''Determines which of the two arms has moved more
        in the recent past'''
        threshold = 0.02
        if (Robot.arms[0].get_movement() < threshold and
                    Robot.arms[1].get_movement() < threshold):
            return -1
        elif (Robot.arms[0].get_movement() < threshold):
            return 1
        else:
            return 0

    @staticmethod
    def get_joint_state(arm_index):
        '''Get joint poritions'''
        return Robot.arms[arm_index].get_joint_state()

    @staticmethod
    def get_gripper_state(arm_index):
        ''' Get gripper status on the indicated side'''
        return Robot.arms[arm_index].get_gripper_state()

    @staticmethod
    def get_gripper_position(arm_index):
        ''' Get gripper status on the indicated side'''
        return Robot.arms[arm_index].get_gripper_position()

    @staticmethod
    def get_ee_state(arm_index):
        ''' Get pose of the end-effector on the indicated side'''
        return Robot.arms[arm_index].get_ee_state()

    @staticmethod
    def _get_time_bw_poses(pose0, pose1, velocity=0.2):
        '''Determines how much time should be allowed for
        moving between two poses'''
        dist = Arm.get_distance_bw_poses(pose0, pose1)
        duration = dist / velocity
        if duration < 0.5:
            duration = 0.5
        return duration

    def update(self):
        '''Periodic update for the two arms'''
        Robot.arms[0].update(self.is_executing())
        Robot.arms[1].update(self.is_executing())

        moving_arm = Robot._get_most_moving_arm()
        if (moving_arm != self.attended_arm and not self.is_executing()):
            if (moving_arm == -1):
                Response.perform_gaze_action(GazeGoal.LOOK_FORWARD)
            elif (moving_arm == 0):
                Response.perform_gaze_action(GazeGoal.FOLLOW_RIGHT_EE)
            else:
                Response.perform_gaze_action(GazeGoal.FOLLOW_LEFT_EE)
            self.attended_arm = moving_arm
