'''Everything related to an experiment session'''
import os
import yaml

import rospy

from pr2_pbd_interaction.msg import ExperimentState
from pr2_pbd_interaction.srv import GetExperimentState
from pr2_pbd_interaction.srv import GetExperimentStateResponse
from step_types import ArmStep
from step_types.ActionReference import ActionReference
from step_types.ManipulationStep import ManipulationStep


class Session:
    '''This class holds and maintains experimental data: list of Actions'''

    session = None

    def __init__(self, object_list):
        self._is_reload = rospy.get_param('/pr2_pbd_interaction/isReload')

        self._exp_number = None
        self._selected_step = 0

        #TODO: read data_dir name from parameters?
        if not os.path.exists(ActionReference.ACTION_DIRECTORY):
            os.makedirs(ActionReference.ACTION_DIRECTORY)
        self.actions = ActionReference.get_saved_actions()
        self.current_action_index = 0 if len(self.actions) > 0 else None

        #link actions in action list to themselves
        for act in self.actions:
            new_steps = []
            for step in act.steps:
                is_step_action = False
                for other_act in self.actions:
                    if other_act.id == step.id:
                        new_steps.append(other_act)
                        is_step_action = True
                        break
                if not is_step_action:
                    new_steps.append(step)
            act.steps = new_steps

            # act.steps = [next((self_act for self_act in self.actions if
            #                    self_act.id == child_act.id), None)
            #              if child_act.type == ActionReference.ACTION_QUEUE else child_act
            #              for child_act in
            #              act.steps]

        # rospy.set_param('data_directory', self._data_dir)
        #
        # self.actions = dict()
        # self.current_action_index = 0
        #
        # if (self._is_reload):
        #     self._load_session_state(object_list)
        #     rospy.loginfo("Session state loaded.")
        #
        # n_actions = dict()
        # for k in self.actions.keys():
        #     n_actions[str(k)] = self.actions[k].n_frames()

        self._state_publisher = rospy.Publisher('experiment_state',
                                                ExperimentState)
        rospy.Service('get_experiment_state', GetExperimentState,
                      self.get_experiment_state_cb)

        self._update_experiment_state()
        Session.session = self


    @staticmethod
    def get_session():
        return Session.session

    def _selected_step_cb(self, selected_step):
        '''Updates the selected step when interactive
        markers are clicked on'''
        self._selected_step = selected_step
        self._update_experiment_state()

    def get_experiment_state_cb(self, dummy):
        ''' Response to the experiment state service call'''
        return GetExperimentStateResponse(self._get_experiment_state())

    def _update_experiment_state(self):
        ''' Publishes a message with the latest state'''
        state = self._get_experiment_state()
        self._state_publisher.publish(state)

    def _get_experiment_state(self):
        ''' Creates a message with the latest state'''
        return ExperimentState(
            (self.actions[self.current_action_index].to_string()
             if self.current_action_index is not None
             else ""),
            map(lambda act: act.name, self.actions),
            map(lambda act: act.id, self.actions),
            0 if self.current_action_index is None else self.current_action_index,
            self._selected_step)

    def _get_ref_frames(self, arm_index):
        ''' Returns the reference frames for the steps of the
        current action in array format'''
        ref_frames = []
        for i in range(self.n_frames()):
            action = self.actions[self.current_action_index]
            ref_frame = action.get_step_ref_frame(arm_index, i)
            ref_frames.append(ref_frame)
        return ref_frames

    def _get_gripper_states(self, arm_index):
        ''' Returns the gripper states for current action
        in array format'''
        gripper_states = []
        for i in range(self.n_frames()):
            action = self.actions[self.current_action_index]
            gripper_state = action.get_step_gripper_state(arm_index, i)
            gripper_states.append(gripper_state)
        return gripper_states

    def select_action_step(self, step_id):
        ''' Makes the interactive marker for the indicated action
        step selected, by showing the 6D controls'''
        self.actions[self.current_action_index].select_step(step_id)
        self._selected_step = step_id

    def deselect_action_step(self, step_id):
        ''' Removes the 6D controls from the interactive marker
        when the indicated action step is deselected'''
        self.actions[self.current_action_index].deselect_step(step_id)
        self._selected_step = 0

    def _get_participant_id(self):
        '''Gets the experiment number from the command line'''
        while (self._exp_number == None):
            try:
                self._exp_number = int(raw_input(
                    'Please enter participant ID:'))
            except ValueError:
                rospy.logerr("Participant ID needs to be a number")

            self._data_dir = Session._get_data_dir(self._exp_number)
            if (not os.path.exists(self._data_dir)):
                os.mkdir(self._data_dir)
            else:
                rospy.logwarn('A directory for this participant ' +
                              'ID already exists: ' + self._data_dir)
                overwrite = raw_input('Do you want to overwrite? ' +
                                      'Type r to reload the last state ' +
                                      'of the experiment. [y/n/r]')
                if (overwrite == 'y'):
                    continue
                elif (overwrite == 'n'):
                    self._exp_number = None
                elif (overwrite == 'r'):
                    self._is_reload = True
                else:
                    rospy.logerr('Invalid response, try again.')

    @staticmethod
    def _get_data_dir(exp_number):
        '''Returns the directory where action information is saved'''
        return (rospy.get_param('/pr2_pbd_interaction/dataRoot') +
                '/data/experiment' + str(exp_number) + '/')


    def save_session_state(self, is_save_actions=True):
        if is_save_actions:
            for i in range(self.n_actions()):
                self.actions[i].save()

    # def new_action(self):
    #     '''Creates new action'''
    #     if (self.n_actions() > 0):
    #         self.get_current_action().reset_viz()
    #     self.current_action_index = self.n_actions() + 1
    #     self.actions.update({self.current_action_index:
    #                              Action(self.current_action_index,
    #                                     self._selected_step_cb)})
    #     self._update_experiment_state()

    def new_action(self):
        '''Creates new action'''
        self.current_action_index = len(self.actions)
        self._selected_step = 0
        newAct = ActionReference(name="Unnamed " + str(self.current_action_index))
        newAct.save()
        self.actions.append(newAct)
        self._update_experiment_state()

    def n_actions(self):
        """Returns the number of actions programmed so far"""
        return len(self.actions)

    def get_current_action(self):
        """Returns the current action"""
        return self.actions[self.current_action_index]

    def get_current_step(self):
        """Returns the current action step"""
        return self.actions[self.current_action_index].get_selected_step()

    def get_last_arm_step(self):
        """ Returns the last arm step if the last action step is ManipulationStep
        and there are arm steps in it, or None otherwise.
        """
        if self.n_actions() > 0:
            last_step = self.get_current_action().get_last_step()
            if last_step is not None and isinstance(last_step, ManipulationStep):
                return last_step.get_last_step()
        return None

    #     def get_current_action_name(self):
    #         return self.actions[self.current_action_index].get_name()

    def clear_current_action(self):
        '''Removes all steps in the current action'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].clear()
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def undo_clear(self):
        '''Undo the effect of clear'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].undoClear()
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def save_current_action(self):
        '''Save current action onto hard drive'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].save(self._data_dir)
            self.save_session_state(is_save_actions=False)
        else:
            rospy.logwarn('No skills created yet.')

    def add_step_to_action(self, step):
        '''Add a new step to the current action'''
        if (self.n_actions() > 0):
            if isinstance(step, ArmStep):
                current_step = self.get_current_step()
                if isinstance(current_step, ManipulationStep):
                    current_step.add_arm_step(step)
                else:
                    new_step = ManipulationStep(self._selected_step_cb)
                    new_step.add_arm_step(step)
                    self.actions[self.current_action_index].add_step(new_step)
            else:
                self.actions[self.current_action_index].add_step(step)
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def add_action_step_action(self, act_name):
        act = next((act for act in self.actions
                if act.name == act_name), None)
        if (act != None):
            self.actions[self.current_action_index].add_step(act)
            self._update_experiment_state()
            return True
        else:
            rospy.logwarn("Action " + act_name + " not found")
            return False

    def delete_last_step(self):
        '''Removes the last step of the action'''
        if (self.n_actions() > 0):
            current_step = self.get_current_step()
            if isinstance(current_step, ManipulationStep):
                current_step.delete_last_step_and_update_viz()
            else:
                self.actions[self.current_action_index].delete_last_step()
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def resume_deleted_step(self):
        '''Resumes the deleted step'''
        if (self.n_actions() > 0):
            self.actions[self.current_action_index].resume_deleted_step()
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def switch_to_action(self, action_number):
        '''Switches to indicated action'''
        if (self.n_actions() > 0):
            if (action_number <= self.n_actions() and action_number > 0):
                self.get_current_action().reset_viz()
                self.current_action_index = action_number
                self.get_current_action().initialize_viz()
                success = True
            else:
                rospy.logwarn('Cannot switch to action '
                              + str(action_number))
                success = False
        else:
            rospy.logwarn('No skills created yet.')
            success = False
        self._update_experiment_state()
        return success

    def switch_to_action_by_name(self, action_name):
        return self.switch_to_action(next((i for i, act in enumerate(self.actions)
                                           if act.name == action_name), -1))

    def name_action(self, new_name):
        if len(self.actions) > 0:
            self.actions[self.current_action_index].name = new_name
            self._update_experiment_state()

    def next_action(self, object_list):
        '''Switches to next action'''
        if (self.n_actions() > 0):
            if (self.current_action_index < self.n_actions()):
                self.get_current_action().reset_viz()
                self.current_action_index += 1
                self.get_current_action().initialize_viz(object_list)
                success = True
            else:
                success = False
        else:
            rospy.logwarn('No skills created yet.')
            success = False
        self._update_experiment_state()
        return success

    def previous_action(self, object_list):
        '''Switches to previous action'''
        if (self.n_actions() > 0):
            if (self.current_action_index > 1):
                self.get_current_action().reset_viz()
                self.current_action_index -= 1
                self.get_current_action().initialize_viz(object_list)
                success = True
            else:
                success = False
        else:
            rospy.logwarn('No skills created yet.')
            success = False
        self._update_experiment_state()
        return success

    def n_frames(self):
        '''Returns the number of frames'''
        if (self.n_actions() > 0):
            return self.actions[self.current_action_index].n_frames()
        else:
            rospy.logwarn('No skills created yet.')
            return 0

    def get_requested_targets(self, arm_index):
        if self.n_actions() > 0:
            cur_step = self.get_current_step()
            if cur_step is not None and isinstance(cur_step, ManipulationStep):
                return cur_step.get_requested_targets(arm_index)
        return None

    def delete_requested_steps(self):
        if self.n_actions() > 0:
            cur_step = self.get_current_step()
            if cur_step is not None and isinstance(cur_step, ManipulationStep):
                cur_step.delete_requested_steps()

    def change_requested_steps(self, r_state, l_state):
        if self.n_actions() > 0:
            cur_step = self.get_current_step()
            if cur_step is not None and isinstance(cur_step, ManipulationStep):
                cur_step.change_requested_steps(r_state, l_state)


