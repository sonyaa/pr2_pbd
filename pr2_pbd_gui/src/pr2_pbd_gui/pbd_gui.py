#!/usr/bin/env python
import functools
import roslib

roslib.load_manifest('pr2_pbd_gui')

import os
import time
from subprocess import call
import rospy, yaml
from std_msgs.msg import String
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui, QtCore
from python_qt_binding.QtGui import QWidget, QFrame
from python_qt_binding.QtGui import QGroupBox, QIcon, QTableView, QStandardItem
from python_qt_binding.QtCore import Slot, qDebug, QSignalMapper, QTimer, qWarning, Signal
from pr2_pbd_interaction.msg import GuiCommand, Strategy
from pr2_pbd_speech_recognition.msg import Command
from sound_play.msg import SoundRequest
from pr2_pbd_interaction.msg import ExperimentState
from pr2_pbd_interaction.srv import GetExperimentState
from step_types.ActionReference import ActionReference
from step_types.BaseStep import BaseStep
from step_types.ManipulationStep import ManipulationStep
from step_types.ObjectDetectionStep import ObjectDetectionStep


class ClickableLabel(QtGui.QLabel):
    def __init__(self, parent, index, clickCallback):
        QtGui.QLabel.__init__(self, parent)
        self.index = index
        self.clickCallback = clickCallback

    def mousePressEvent(self, event):
        self.emit(QtCore.SIGNAL('clicked()'), "Label pressed")
        self.clickCallback(self.index)


class ActionIcon(QtGui.QGridLayout):
    def __init__(self, parent, index, name, clickCallback):
        QtGui.QGridLayout.__init__(self)
        self.setSpacing(0)
        path = os.popen('rospack find pr2_pbd_gui').read()
        path = path[0:len(path) - 1]
        self.notSelectedIconPath = path + '/icons/actions0.png'
        self.selectedIconPath = path + '/icons/actions1.png'
        self.selected = False
        self.actionIconWidth = 50
        self.index = index
        self.icon = ClickableLabel(parent, index, clickCallback)
        self.text = QtGui.QLabel(parent)
        self.text.setText(name)
        self.updateView()
        self.addWidget(self.icon, 0, 0, QtCore.Qt.AlignCenter)
        self.addWidget(self.text, 1, 0, QtCore.Qt.AlignCenter)

    def updateView(self):
        if self.selected:
            pixmap = QtGui.QPixmap(self.selectedIconPath)
        else:
            pixmap = QtGui.QPixmap(self.notSelectedIconPath)
        self.icon.setPixmap(pixmap.scaledToWidth(self.actionIconWidth, QtCore.Qt.SmoothTransformation))


class PbDGUI(Plugin):
    exp_state_sig = Signal(ExperimentState)

    def __init__(self, context):
        super(PbDGUI, self).__init__(context)
        self.recording = False

        self.setObjectName('PbDGUI')
        self._widget = QWidget()

        self.speech_cmd_publisher = rospy.Publisher('recognized_command', Command)
        self.gui_cmd_publisher = rospy.Publisher('gui_command', GuiCommand)

        rospy.Subscriber('experiment_state', ExperimentState, self.exp_state_cb)
        rospy.Subscriber('robotsound', SoundRequest, self.robotSoundReceived)

        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.exp_state_sig.connect(self.update_state)

        self.commands = dict()
        self.commands[Command.CREATE_NEW_ACTION] = 'New action'
        self.commands[Command.TEST_MICROPHONE] = 'Test microphone'
        self.commands[Command.NEXT_ACTION] = 'Next action'
        self.commands[Command.PREV_ACTION] = 'Previous action'
        self.commands[Command.SAVE_POSE] = 'Save arm pose'
        self.commands[Command.SAVE_BASE_POSE] = 'Save base pose'
        #adding record motion
        self.commands[Command.START_RECORDING_MOTION] = 'Record motion'
        self.commands[Command.STOP_RECORDING_MOTION] = 'Stop recording motion'

        self.commands[Command.RELAX_RIGHT_ARM] = 'Relax right arm'
        self.commands[Command.RELAX_LEFT_ARM] = 'Relax left arm'
        self.commands[Command.FREEZE_RIGHT_ARM] = 'Freeze right arm'
        self.commands[Command.FREEZE_LEFT_ARM] = 'Freeze left arm'
        self.commands[Command.OPEN_RIGHT_HAND] = 'Open right hand'
        self.commands[Command.OPEN_LEFT_HAND] = 'Open left hand'
        self.commands[Command.CLOSE_RIGHT_HAND] = 'Close right hand'
        self.commands[Command.CLOSE_LEFT_HAND] = 'Close left hand'
        self.commands[Command.CLOSE_LEFT_HAND] = 'Close left hand'
        self.commands[Command.EXECUTE_ACTION] = 'Execute action'
        self.commands[Command.STOP_EXECUTION] = 'Stop execution'
        self.commands[Command.DELETE_ALL_STEPS] = 'Delete all'
        self.commands[Command.DELETE_LAST_ARM_STEP] = 'Delete last'
        #self.commands[Command.REPEAT_LAST_STEP] = 'Repeat last step'
        self.commands[Command.RECORD_OBJECT_POSE] = 'Record object poses'
        self.commands[Command.SAVE_ACTION] = 'Save action'

        self.currentAction = -1
        self.currentStep = -1
        self.selectedArmStepUid = -1
        self.action = None
        self.is_edit = False

        self.editingBox = QtGui.QVBoxLayout()
        allWidgetsBox = QtGui.QVBoxLayout()
        actionBox = QGroupBox('Actions', self._widget)
        self.actionGrid = QtGui.QGridLayout()
        self.actionGrid.setHorizontalSpacing(0)
        for i in range(6):
            self.actionGrid.addItem(QtGui.QSpacerItem(90, 90), 0, i, QtCore.Qt.AlignCenter)
            self.actionGrid.setColumnStretch(i, 0)
        self.actionIcons = dict()
        actionBoxLayout = QtGui.QHBoxLayout()
        actionBoxLayout.addLayout(self.actionGrid)
        actionBox.setLayout(actionBoxLayout)

        actionButtonGrid = QtGui.QHBoxLayout()
        actionButtonGrid.addWidget(self.create_button(
            Command.CREATE_NEW_ACTION))
        self.stepsGroupBox = QGroupBox('Action steps', self._widget)
        self.stepsVBox = QtGui.QVBoxLayout()

        stepsBoxLayout = QtGui.QHBoxLayout()
        stepsBoxLayout.addLayout(self.stepsVBox)
        self.stepsGroupBox.setLayout(stepsBoxLayout)

        stepsButtonGrid = QtGui.QHBoxLayout()
        #stepsButtonGrid.addWidget(self.create_button(Command.SAVE_POSE))
        stepsButtonGrid.addWidget(self.create_button(Command.EXECUTE_ACTION))
        stepsButtonGrid.addWidget(self.create_button(Command.STOP_EXECUTION))
        #stepsButtonGrid.addWidget(self.create_button(Command.DELETE_ALL_STEPS))
        #stepsButtonGrid.addWidget(self.create_button(Command.DELETE_LAST_STEP))
        #stepsButtonGrid.addWidget(self.create_button(Command.REPEAT_LAST_STEP))

        actionNameGrid = QtGui.QHBoxLayout()
        actionNameGrid.addWidget(QtGui.QLabel("Action name:", self._widget))
        self.act_name_box = QtGui.QLineEdit(self._widget)
        actionNameGrid.addWidget(self.act_name_box)
        ch_name_but = QtGui.QPushButton("Change", self._widget)

        def change_name():
            self.speech_cmd_publisher.publish(Command(Command.CHANGE_ACTION_NAME + " " +
                                                      self.act_name_box.text()))

        ch_name_but.clicked.connect(change_name)
        actionNameGrid.addWidget(ch_name_but)

        misc_grid = QtGui.QHBoxLayout()
        misc_grid.addWidget(self.create_button(Command.TEST_MICROPHONE))
        misc_grid.addWidget(self.create_button(Command.RECORD_OBJECT_POSE))
        misc_grid.addWidget(self.create_button(Command.SAVE_BASE_POSE))
        misc_grid.addWidget(self.create_button(Command.SAVE_POSE))
        misc_grid.addStretch(1)

        misc_grid2 = QtGui.QHBoxLayout()
        misc_grid2.addWidget(self.create_button(Command.RELAX_RIGHT_ARM))
        misc_grid2.addWidget(self.create_button(Command.RELAX_LEFT_ARM))
        misc_grid2.addWidget(self.create_button(Command.FREEZE_RIGHT_ARM))
        misc_grid2.addWidget(self.create_button(Command.FREEZE_LEFT_ARM))
        misc_grid2.addStretch(1)

        misc_grid3 = QtGui.QHBoxLayout()
        misc_grid3.addWidget(self.create_button(Command.OPEN_RIGHT_HAND))
        misc_grid3.addWidget(self.create_button(Command.OPEN_LEFT_HAND))
        misc_grid3.addWidget(self.create_button(Command.CLOSE_RIGHT_HAND))
        misc_grid3.addWidget(self.create_button(Command.CLOSE_LEFT_HAND))
        misc_grid3.addStretch(1)

        misc_grid4 = QtGui.QHBoxLayout()
        misc_grid4.addWidget(self.create_button(Command.PREV_ACTION))
        misc_grid4.addWidget(self.create_button(Command.NEXT_ACTION))
        misc_grid4.addWidget(self.create_button(Command.SAVE_ACTION))
        misc_grid4.addStretch(1)

        speechGroupBox = QGroupBox('Robot Speech', self._widget)
        speechGroupBox.setObjectName('RobotSpeechGroup')
        speechBox = QtGui.QHBoxLayout()
        self.speechLabel = QtGui.QLabel('Robot has not spoken yet')
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Foreground, QtCore.Qt.blue)
        self.speechLabel.setPalette(palette)
        speechBox.addWidget(self.speechLabel)
        speechGroupBox.setLayout(speechBox)

        allWidgetsBox.addWidget(actionBox)
        allWidgetsBox.addLayout(actionButtonGrid)

        allWidgetsBox.addWidget(self.stepsGroupBox)
        allWidgetsBox.addLayout(stepsButtonGrid)
        allWidgetsBox.addLayout(actionNameGrid)
        #allWidgetsBox.addLayout(motionButtonGrid)

        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addLayout(misc_grid)
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addLayout(misc_grid2)
        allWidgetsBox.addLayout(misc_grid3)
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))
        allWidgetsBox.addLayout(misc_grid4)
        allWidgetsBox.addItem(QtGui.QSpacerItem(100, 20))

        allWidgetsBox.addWidget(speechGroupBox)
        allWidgetsBox.addStretch(1)

        # Fix layout and add main widget to the user interface
        QtGui.QApplication.setStyle(QtGui.QStyleFactory.create('plastique'))
        vAllBox = QtGui.QVBoxLayout()
        vAllBox.addLayout(allWidgetsBox)
        vAllBox.addStretch(1)
        hAllBox = QtGui.QHBoxLayout()
        hAllBox.addLayout(vAllBox)
        hAllBox.addLayout(self.editingBox)
        hAllBox.addStretch(1)

        self._widget.setObjectName('PbDGUI')
        self._widget.setLayout(hAllBox)
        context.add_widget(self._widget)

        rospy.loginfo('Will wait for the experiment state service...')
        rospy.wait_for_service('get_experiment_state')
        exp_state_srv = rospy.ServiceProxy('get_experiment_state',
                                           GetExperimentState)
        rospy.loginfo('Got response from the experiment state service...')

        response = exp_state_srv()
        self.update_state(response.state)

    @staticmethod
    def _set_enabled_widgets_in_layout(layout, enable=True):
        for i in range(layout.count()):
            widget = layout.itemAt(i).widget()
            if widget is not None:
                widget.setEnabled(enable)

    def get_uid(self, arm_index, index):
        '''Returns a unique id of the marker'''
        return (2 * (index + 1) + arm_index)

    def get_arm_and_index(self, uid):
        '''Returns a unique id of the marker'''
        arm_index = uid % 2
        index = (uid - arm_index) / 2
        return (arm_index, (index - 1))

    def create_button(self, command):
        btn = QtGui.QPushButton(self.commands[command], self._widget)
        btn.clicked.connect(self.command_cb)
        return btn

    def show_action(self, act_name):
        for ind, name in enumerate(self.action_names):
            if name == act_name:
                self.action_pressed(ind)
                return
        rospy.logwarn("Couldn't find action with name " + act_name)

    def disp_action(self, act):
        self.act_name_box.setText(act.name)
        while self.stepsVBox.count() > 0:
            layout = self.stepsVBox.itemAt(0).layout()
            self.stepsVBox.removeItem(layout)
            if layout is not None:
                while layout.count() > 0:
                    widget = layout.itemAt(0).widget()
                    layout.removeWidget(widget)
                    if widget is not None:
                        widget.deleteLater()
                        del widget
                del layout
        for ind, sub_act in enumerate(act.steps):
            typeLabel = QtGui.QLabel(self._widget)
            has_edit = True
            viewBtn = QtGui.QPushButton("View", self._widget)
            viewBtn.clicked.connect(functools.partial(self.step_pressed, ind))
            if ind == self.currentStep:
                viewBtn.setStyleSheet('QPushButton {font-weight: bold}')
            if isinstance(sub_act, ManipulationStep):
                typeLabel.setText("Manipulation")
            elif isinstance(sub_act, BaseStep):
                typeLabel.setText("Navigation")
            elif isinstance(sub_act, ObjectDetectionStep):
                typeLabel.setText("Object detection")
                viewBtn.setEnabled(False)
            elif isinstance(sub_act, ActionReference):
                typeLabel.setText("Preprogrammed: " + sub_act.name)
                viewBtn = QtGui.QPushButton("Show", self._widget)
                viewBtn.clicked.connect(functools.partial(self.show_action, sub_act.name))
                has_edit = False
            else:
                rospy.logwarn("Unknown action step type " + str(type(sub_act)))
                continue
            stepRow = QtGui.QHBoxLayout()
            stepRow.addWidget(typeLabel)
            stepRow.addWidget(viewBtn)
            if has_edit:
                editBtn = QtGui.QPushButton("Edit", self._widget)
                editBtn.clicked.connect(functools.partial(self.edit_button_pressed, ind))
                stepRow.addWidget(editBtn)
            self.stepsVBox.addLayout(stepRow)
        add_action_row = QtGui.QHBoxLayout()
        self.action_selector = QtGui.QComboBox(self._widget)
        add_action_row.addWidget(QtGui.QLabel("Add action step:"))
        for name in self.action_names:
            if name != act.name:
                self.action_selector.addItem(name)
        add_action_row.addWidget(self.action_selector)
        add_btn = QtGui.QPushButton("Add step", self._widget)
        add_btn.clicked.connect(self.add_action_step)
        add_action_row.addWidget(add_btn)
        self.stepsVBox.addLayout(add_action_row)
        if self.is_edit:
            self.display_editing_area()
        else:
            self.clear_editing_area()
        self.update_action_steps_buttons()

    def add_action_step(self):
        self.speech_cmd_publisher.publish(Command(Command.ADD_ACTION_STEP + " " +
                                                  self.action_selector.currentText()))

    def update_state(self, state):
        qWarning('Received new state')
        rospy.loginfo(state)
        ## TODO: DEAL with the following arrays!!!
        #state.r_gripper_states
        #state.l_gripper_states
        #state.r_ref_frames
        #state.l_ref_frames
        #state.objects

        # If the state changed a substep of the current action, leave is_edit as is.
        # Otherwise, only display editing area after an explicit click on the Edit button.
        if self.currentAction != state.selected_action:
            self.is_edit = False
        self.currentAction = state.selected_action
        self.currentStep = state.selected_step
        self.selectedArmStepUid = state.selected_arm_step
        self.action_ids = state.action_ids
        self.action_names = state.action_names
        nColumns = 6
        # Delete previous action icons
        for icon in self.actionIcons.values():
            while icon.count():
                item = icon.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.deleteLater()
                else:
                    self.clearLayout(item.layout())
            self.actionGrid.removeItem(icon)
            icon.deleteLater()
            del icon
        self.actionIcons.clear()
        for index, action_name in enumerate(state.action_names):
            actIcon = ActionIcon(self._widget, index, action_name, self.action_pressed)
            self.actionGrid.addLayout(actIcon, int(index / nColumns),
                                      index % nColumns)
            self.actionIcons[index] = actIcon
            if (index == state.selected_action):
                actIcon.selected = True
                actIcon.updateView()
        if (state.action_str != ""):
            self.action = ActionReference.from_string(state.action_str)  #state['action_str'])
            self.disp_action(self.action)

    def n_actions(self):
        return len(self.actionIcons.keys())

    def update_action_steps_buttons(self):
        # Make text on buttons for the current step bold.
        for ind_l in xrange(self.stepsVBox.count()):
            layout = self.stepsVBox.itemAt(ind_l).layout()
            for ind_w in xrange(layout.count()):
                widget = layout.itemAt(ind_w).widget()
                if widget is not None and isinstance(widget, QtGui.QPushButton):
                    if widget.text() == "View":
                        if ind_l == self.currentStep:
                            widget.setStyleSheet('QPushButton {font-weight: bold}')
                        else:
                            widget.setStyleSheet('QPushButton {font-weight: normal}')
                    if widget.text() == "Edit":
                        if self.is_edit and ind_l == self.currentStep:
                            widget.setStyleSheet('QPushButton {font-weight: bold}')
                        else:
                            widget.setStyleSheet('QPushButton {font-weight: normal}')

    def display_editing_area(self):
        self.clear_editing_area()
        if self.action is not None and 0 <= self.currentStep < len(self.action.steps):
            step = self.action.steps[self.currentStep]
            header_layout = QtGui.QHBoxLayout()
            self.editingBox.addLayout(header_layout)
            typeLabel = QtGui.QLabel(self._widget)
            header_text = "Editing %s step"
            if isinstance(step, ManipulationStep):
                typeLabel.setText(header_text % "Manipulation")
                arm_steps_grid = QtGui.QGridLayout()
                self.editingBox.addLayout(arm_steps_grid)
                arm_steps_grid.addWidget(QtGui.QLabel('Right arm', self._widget), 0, 0)
                arm_steps_grid.addWidget(QtGui.QLabel('Position', self._widget), 0, 1)
                arm_steps_grid.setColumnMinimumWidth(2, 10)
                arm_steps_grid.setColumnMinimumWidth(3, 10)
                arm_steps_grid.addWidget(QtGui.QLabel('Left arm', self._widget), 0, 4)
                arm_steps_grid.addWidget(QtGui.QLabel('Position', self._widget), 0, 5)
                arm_steps_grid.setColumnMinimumWidth(6, 10)
                arm_steps_grid.setColumnMinimumWidth(7, 10)
                arm_steps_grid.setColumnMinimumWidth(8, 10)
                for ind, arm_step in enumerate(step.arm_steps):
                    abs_string_r = 'Relative' if arm_step.is_relative(0) else 'Absolute'
                    abs_string_l = 'Relative' if arm_step.is_relative(1) else 'Absolute'
                    del_pose_button = QtGui.QPushButton('Delete', self._widget)
                    del_pose_button.clicked.connect(functools.partial(self.delete_arm_step, ind))
                    view_r_button = QtGui.QPushButton('Select', self._widget)
                    view_r_button.clicked.connect(functools.partial(self.arm_step_pressed, self.get_uid(0, ind)))
                    view_l_button = QtGui.QPushButton('Select', self._widget)
                    view_l_button.clicked.connect(functools.partial(self.arm_step_pressed, self.get_uid(1, ind)))
                    arm_steps_grid.addWidget(QtGui.QLabel('Step ' + str(ind + 1), self._widget), ind + 1, 0)
                    arm_steps_grid.addWidget(QtGui.QLabel(abs_string_r, self._widget), ind + 1, 1)
                    arm_steps_grid.addWidget(view_r_button, ind + 1, 2)
                    arm_steps_grid.addWidget(QtGui.QLabel('Step ' + str(ind + 1), self._widget), ind + 1, 4)
                    arm_steps_grid.addWidget(QtGui.QLabel(abs_string_l, self._widget), ind + 1, 5)
                    arm_steps_grid.addWidget(view_l_button, ind + 1, 6)
                    arm_steps_grid.addWidget(del_pose_button, ind + 1, 8)
                    # TODO: add editing conditions for arm steps
            elif isinstance(step, BaseStep):
                typeLabel.setText(header_text % "Navigation")
            elif isinstance(step, ObjectDetectionStep):
                typeLabel.setText(header_text % "Object detection")
            else:
                rospy.logwarn("Unknown action step type " + str(type(step)))
                return
            delete_button = QtGui.QPushButton("Delete step", self._widget)
            delete_button.clicked.connect(functools.partial(self.delete_step, self.currentStep))
            header_layout.addWidget(typeLabel)
            header_layout.addWidget(delete_button)
            conditions_layout = QtGui.QHBoxLayout()
            conditions_layout.addWidget(QtGui.QLabel("Step has " + str(len(step.conditions)) + " conditions."))
            conditions_edit_btn = QtGui.QPushButton('Edit conditions', self._widget)
            conditions_edit_btn.clicked.connect(self.edit_conditions)
            conditions_layout.addWidget(conditions_edit_btn)
            self.editingBox.addLayout(conditions_layout)
            strategy_layout = QtGui.QHBoxLayout()
            strategy_layout.addWidget(QtGui.QLabel("If condition fails:", self._widget))
            strategy_selector = QtGui.QComboBox(self._widget)
            strategy_selector.addItem("Fail-fast")
            strategy_selector.addItem("Continue")
            if step.strategy == Strategy.FAIL_FAST:
                strategy_selector.setCurrentIndex(0)
            elif step.strategy == Strategy.CONTINUE:
                strategy_selector.setCurrentIndex(1)
            strategy_selector.currentIndexChanged.connect(self.change_strategy)
            strategy_layout.addWidget(strategy_selector)
            self.editingBox.addLayout(strategy_layout)
            while_layout = QtGui.QHBoxLayout()
            is_loop_checkbox = QtGui.QCheckBox("Make this a 'while' loop", self._widget)
            if step.is_while:
                is_loop_checkbox.setChecked(True)
            is_loop_checkbox.clicked.connect(self.set_loop)
            while_layout.addWidget(is_loop_checkbox)
            self.editingBox.addLayout(while_layout)

    def clear_editing_area(self):
        while self.editingBox.count() > 0:
            layout = self.editingBox.itemAt(0).layout()
            self.editingBox.removeItem(layout)
            if layout is not None:
                while layout.count() > 0:
                    widget = layout.itemAt(0).widget()
                    layout.removeWidget(widget)
                    if widget is not None:
                        widget.deleteLater()
                        del widget
                del layout

    def edit_conditions(self):
        pass

    def set_loop(self, is_checked):
        if is_checked:
            gui_cmd = GuiCommand(GuiCommand.SET_LOOP_STEP, self.currentStep)
            self.gui_cmd_publisher.publish(gui_cmd)
        else:
            gui_cmd = GuiCommand(GuiCommand.SET_NO_LOOP_STEP, self.currentStep)
            self.gui_cmd_publisher.publish(gui_cmd)

    def change_strategy(self, index):
        if index == 0:
            param = Strategy.FAIL_FAST
        elif index == 1:
            param = Strategy.CONTINUE
        else:
            rospy.logwarn('Unknown strategy selector index ' + str(index))
            return
        gui_cmd = GuiCommand(GuiCommand.SET_STRATEGY, param)
        self.gui_cmd_publisher.publish(gui_cmd)

    def edit_button_pressed(self, step_index):
        self.step_pressed(step_index)
        if self.currentStep == step_index:
            if self.is_edit:
                self.is_edit = False
                self.clear_editing_area()
            else:
                self.is_edit = True
                self.display_editing_area()
            self.update_action_steps_buttons()
        else:
            self.is_edit = True
            self.display_editing_area()

    def step_pressed(self, step_index):
        if self.currentStep != step_index:
            self.is_edit = False
            self.clear_editing_area()
            self.currentStep = step_index
            gui_cmd = GuiCommand(GuiCommand.SELECT_ACTION_STEP, step_index)
            self.gui_cmd_publisher.publish(gui_cmd)
            self.update_action_steps_buttons()

    def delete_step(self, index):
        gui_cmd = GuiCommand(GuiCommand.DELETE_STEP, index)
        self.gui_cmd_publisher.publish(gui_cmd)

    def delete_arm_step(self, index):
        gui_cmd = GuiCommand(GuiCommand.DELETE_ARM_STEP, index)
        self.gui_cmd_publisher.publish(gui_cmd)

    def arm_step_pressed(self, step_index):
        if step_index != self.selectedArmStepUid:
            self.selectedArmStepUid = step_index
            gui_cmd = GuiCommand(GuiCommand.SELECT_ARM_STEP, step_index)
        else:
            self.selectedArmStepUid = -1
            gui_cmd = GuiCommand(GuiCommand.DESELECT_ARM_STEP, step_index)
        self.gui_cmd_publisher.publish(gui_cmd)

    def action_pressed(self, actionIndex, isPublish=True):
        if isPublish:
            gui_cmd = GuiCommand(GuiCommand.SWITCH_TO_ACTION, actionIndex)
            self.gui_cmd_publisher.publish(gui_cmd)

    def command_cb(self):
        clickedButtonName = self._widget.sender().text()
        for key in self.commands.keys():
            if (self.commands[key] == clickedButtonName):
                qWarning('Sending speech command: ' + key)
                command = Command()
                command.command = key
                self.speech_cmd_publisher.publish(command)

    def robotSoundReceived(self, soundReq):
        if (soundReq.command == SoundRequest.SAY):
            qWarning('Robot said: ' + soundReq.arg)
            self.speechLabel.setText('Robot sound: ' + soundReq.arg)

    def exp_state_cb(self, state):
        qWarning('Received new experiment state.')
        self.exp_state_sig.emit(state)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.speech_cmd_publisher.unregister()
        self.gui_cmd_publisher.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
