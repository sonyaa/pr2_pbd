#!/usr/bin/env python
import roslib
roslib.load_manifest('pr2_pbd_gui')

import os
import time
from subprocess import call
import rospy, yaml
from std_msgs.msg import String
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame
from python_qt_binding.QtGui import QGroupBox, QIcon, QTableView, QStandardItem
from python_qt_binding.QtCore import Slot, qDebug, QSignalMapper, QTimer, qWarning, Signal
from pr2_pbd_interaction.msg import GuiCommand
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
        path = path[0:len(path)-1]
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
        self.commands[Command.DELETE_LAST_STEP] = 'Delete last'
        #self.commands[Command.REPEAT_LAST_STEP] = 'Repeat last step'
        self.commands[Command.RECORD_OBJECT_POSE] = 'Record object poses'
        self.commands[Command.SAVE_ACTION] = 'Save action'
        
        self.currentAction = -1
        self.currentStep = -1

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
        
        #self.l_model = QtGui.QStandardItemModel(self)
        #self.l_view = self._create_table_view(self.l_model,
                                              #self.l_row_clicked_cb)
        #self.r_model = QtGui.QStandardItemModel(self)
        #self.r_view = self._create_table_view(self.r_model,
                                              #self.r_row_clicked_cb)

        #self.stepsGrid.addItem(QtGui.QSpacerItem(280, 10), 0, 0, 2, 3)
        #self.stepsGrid.addItem(QtGui.QSpacerItem(10, 10), 0, 1, 2, 3)
        #self.stepsGrid.addItem(QtGui.QSpacerItem(280, 10), 0, 2, 2, 3)
        
        #self.stepsGrid.addWidget(QtGui.QLabel('Left Arm'), 0, 0)
        #self.stepsGrid.addWidget(QtGui.QLabel('Right Arm'), 0, 2)

        #self.stepsGrid.addWidget(self.l_view, 1, 0)
        #self.stepsGrid.addWidget(self.r_view, 1, 2)
        
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
        
        #motionButtonGrid = QtGui.QHBoxLayout()
        #motionButtonGrid.addWidget(self.create_button(Command.START_RECORDING_MOTION))
        #motionButtonGrid.addWidget(self.create_button(Command.START_RECORDING_RELATIVE_MOTION))
        #motionButtonGrid.addWidget(self.create_button(Command.STOP_RECORDING_MOTION))
        
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
        palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.blue)
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
        #response =  { "action_str" : '<action id="None" inline="True" type="0"><name>Action 1</name><actions><action inline="True" type="1"><name /><pose><arms><arm index="0"><position><x>10.0</x><y>2.0</y><z>5.0</z></position><orientation><x>1.0</x><y>1.0</y><z>5.0</z><w>9.0</w></orientation></arm><arm index="1"><position><x>10.0</x><y>2.0</y><z>5.0</z></position><orientation><x>1.0</x><y>1.0</y><z>5.0</z><w>9.0</w></orientation></arm></arms></pose><target><type_id>1</type_id></target></action></actions></action>' }
        self.update_state(response.state)
        
    @staticmethod
    def _set_enabled_widgets_in_layout(layout, enable=True):
        for i in range(layout.count()):
            widget = layout.itemAt(i).widget()
            if widget is not None:
                widget.setEnabled(enable)

    def _create_table_view(self, model, row_click_cb):
        proxy = QtGui.QSortFilterProxyModel(self)
        proxy.setSourceModel(model)
        view = QtGui.QTableView(self._widget)
        verticalHeader = view.verticalHeader()
        verticalHeader.sectionClicked.connect(row_click_cb)
        view.setModel(proxy)
        view.setMaximumWidth(250)
        view.setSortingEnabled(False)
        view.setCornerButtonEnabled(False)
        return view
    
    def get_uid(self, arm_index, index):
        '''Returns a unique id of the marker'''
        return (2 * (index + 1) + arm_index)
    
    def get_arm_and_index(self, uid):
        '''Returns a unique id of the marker'''
        arm_index = uid % 2
        index = (uid - arm_index) / 2
        return (arm_index, (index - 1))

    def r_row_clicked_cb(self, logicalIndex):
        self.arm_step_pressed(self.get_uid(0, logicalIndex))

    def l_row_clicked_cb(self, logicalIndex):
        self.arm_step_pressed(self.get_uid(1, logicalIndex))

    def create_button(self, command):
        btn = QtGui.QPushButton(self.commands[command], self._widget)
        btn.clicked.connect(self.command_cb)
        return btn

    def view_manipulation(self):
        pass

    def edit_manipulation(self):
        pass

    def view_navigation(self):
        pass

    def edit_navigation(self):
        pass

    def view_object_detection(self):
        pass

    def edit_object_detection(self):
        pass

    def show_action(self, act_name):
        def switch_to_action():
            pass
        return switch_to_action()
        
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
            editBtn = None
            if isinstance(sub_act, ManipulationStep):
                typeLabel = QtGui.QLabel(self._widget)
                typeLabel.setText("Manipulation")
                viewBtn = QtGui.QPushButton("View", self._widget)
                viewBtn.clicked.connect(self.view_manipulation)
                editBtn = QtGui.QPushButton("Edit", self._widget)
                editBtn.clicked.connect(self.edit_manipulation)
            elif isinstance(sub_act, BaseStep):
                typeLabel = QtGui.QLabel(self._widget)
                typeLabel.setText("Navigation")
                viewBtn = QtGui.QPushButton("View", self._widget)
                viewBtn.clicked.connect(self.view_navigation)
                editBtn = QtGui.QPushButton("Edit", self._widget)
                editBtn.clicked.connect(self.edit_navigation)
            elif isinstance(sub_act, ObjectDetectionStep):
                typeLabel = QtGui.QLabel(self._widget)
                typeLabel.setText("Object detection")
                viewBtn = QtGui.QPushButton("View", self._widget)
                viewBtn.clicked.connect(self.view_object_detection)
                editBtn = QtGui.QPushButton("Edit", self._widget)
                editBtn.clicked.connect(self.edit_object_detection)
            elif isinstance(sub_act, ActionReference):
                typeLabel = QtGui.QLabel(self._widget)
                typeLabel.setText("Preprogrammed: " + sub_act.name)
                viewBtn = QtGui.QPushButton("Show", self._widget)
                viewBtn.clicked.connect(self.show_action(sub_act.name))
            else:
                rospy.logwarn("Unknown action step type " + str(type(sub_act)))
                continue
            stepRow = QtGui.QHBoxLayout()
            stepRow.addWidget(typeLabel)
            stepRow.addWidget(viewBtn)
            if editBtn is not None:
                stepRow.addWidget(editBtn)
            self.stepsVBox.addLayout(stepRow)
            # def createLayout(ind, sub_act):
            #     stepRow = ind + 1
            #     typeBox = QtGui.QComboBox(self._widget)
            #     typeBox.addItem("Action")
            #     typeBox.addItem("Pose")
            #     typeBox.addItem("Trajectory")
            #
            #     def type_changed():
            #         setRow(3 if typeBox.currentIndex() == 2 else typeBox.currentIndex())
            #
            #     typeBox.currentIndexChanged.connect(type_changed)
            #     self.stepsGrid.addWidget(typeBox, stepRow, 0)
            #
            #     def setRow(act_type):
            #         for itm in [self.stepsGrid.itemAtPosition(stepRow, i)
            #                 for i in range(1, 4)]:
            #             if itm:
            #                 wid = itm.widget()
            #                 self.stepsGrid.removeWidget(wid)
            #                 wid.deleteLater()
            #                 del wid
            #         if (act_type == Action.ACTION_QUEUE):
            #             act_name = QtGui.QComboBox(self._widget)
            #             act_name.addItems(self.action_names)
            #             if (sub_act.id != None):
            #                 act_name.setCurrentIndex(self.action_ids.index(sub_act.id))
            #             self.stepsGrid.addWidget(act_name, stepRow, 1)
            #             save_but = QtGui.QPushButton("Save", self._widget)
            #             def change_act():
            #                 self.gui_cmd_publisher.publish(
            #                         GuiCommand(GuiCommand.SELECT_ACTION_STEP, ind + 1))
            #                 self.speech_cmd_publisher.publish(Command(Command.DELETE_LAST_STEP))
            #                 self.speech_cmd_publisher.publish(Command(
            #                     Command.ADD_ACTION_STEP + " " + self.action_names[
            #                         act_name.currentIndex()]))
            #             save_but.clicked.connect(change_act)
            #             self.stepsGrid.addWidget(save_but, stepRow, 2)
            #         elif (act_type == Action.POSE):
            #             rec_but = QtGui.QPushButton("Record", self._widget)
            #             def hand_rec():
            #                 self.gui_cmd_publisher.publish(
            #                         GuiCommand(GuiCommand.SELECT_ACTION_STEP, ind + 1))
            #                 self.speech_cmd_publisher.publish(Command(Command.DELETE_LAST_STEP))
            #                 self.speech_cmd_publisher.publish(Command(Command.SAVE_POSE))
            #             rec_but.clicked.connect(hand_rec)
            #             self.stepsGrid.addWidget(rec_but, stepRow, 1)
            #         elif (act_type == Action.TRAJECTORY):
            #             rec_but = QtGui.QPushButton("Record", self._widget)
            #             def hand_rec():
            #                 if (self.recording):
            #                     rec_but.setText("Record")
            #                     self.recording = False
            #                     self.speech_cmd_publisher.publish(Command(
            #                             Command.STOP_RECORDING_MOTION))
            #                     # rospy.loginfo("would have deleted at " + str(ind + 1))
            #                     #self.gui_cmd_publisher.publish(
            #                             #GuiCommand(GuiCommand.SELECT_ACTION_STEP, ind + 1))
            #                     #self.speech_cmd_publisher.publish(Command(
            #                             #Command.DELETE_LAST_STEP))
            #                 elif (not (self.recording)):
            #                     rec_but.setText("Stop")
            #                     self.recording = True
            #                     self.gui_cmd_publisher.publish(
            #                             GuiCommand(GuiCommand.SELECT_ACTION_STEP, ind + 1))
            #                     self.speech_cmd_publisher.publish(Command(
            #                             Command.START_RECORDING_MOTION))
            #             rec_but.clicked.connect(hand_rec)
            #             self.stepsGrid.addWidget(rec_but, stepRow, 1)
            #
            #     typeBox.setCurrentIndex(2 if sub_act.type == Action.TRAJECTORY else sub_act.type)
            #     if (sub_act.type == 0):
            #         setRow(0)
            #
            #     btnPls = QtGui.QPushButton("+", self._widget)
            #     def pls_clicked():
            #         self.gui_cmd_publisher.publish(
            #                 GuiCommand(GuiCommand.SELECT_ACTION_STEP, ind + 1))
            #         self.speech_cmd_publisher.publish(Command(Command.SAVE_POSE))
            #     btnPls.clicked.connect(pls_clicked)
            #     self.stepsGrid.addWidget(btnPls, stepRow, 5)
            #     btnMns = QtGui.QPushButton("-", self._widget)
            #     def mns_clicked():
            #         self.gui_cmd_publisher.publish(
            #                 GuiCommand(GuiCommand.SELECT_ACTION_STEP, ind + 1))
            #         self.speech_cmd_publisher.publish(Command(
            #                 Command.DELETE_LAST_STEP))
            #     btnMns.clicked.connect(mns_clicked)
            #     self.stepsGrid.addWidget(btnMns, stepRow, 4)
            # createLayout(ind, sub_act)
        
    def update_state(self, state):
        qWarning('Received new state')
        rospy.loginfo(state)
        #n_actions = len(self.actionIcons.keys())
        #if n_actions < state.n_actions:
            #for i in range(n_actions, state.n_actions):
                #self.new_action()

        #if (self.currentAction != (state.i_current_action - 1)):
            #self.delete_all_steps()
            #self.action_pressed(state.i_current_action - 1, False)

        #n_steps = self.n_steps()
        #if (n_steps < state.n_steps):
            #for i in range(n_steps, state.n_steps):
                #self.save_pose(frameType=ord(state.frame_types[i]))
        #elif (n_steps > state.n_steps):
            #n_to_remove = n_steps - state.n_steps
            #self.r_model.invisibleRootItem().removeRows(state.n_steps,
                                                      #n_to_remove)
            #self.l_model.invisibleRootItem().removeRows(state.n_steps,
                                                      #n_to_remove)
        
        ## TODO: DEAL with the following arrays!!!
        #state.r_gripper_states
        #state.l_gripper_states
        #state.r_ref_frames
        #state.l_ref_frames
        #state.objects
            
        #if (self.currentStep != state.i_current_step):
            #if (self.n_steps() > 0):
                #self.currentStep = state.i_current_step
                #arm_index, index = self.get_arm_and_index(self.currentStep)
                #if (arm_index == 0):
                    #self.r_view.selectRow(index)
                #else:
                    #self.l_view.selectRow(index)
        
        '''New code to deal with xml-ed actions'''
        self.action_ids = state.action_ids
        self.action_names = state.action_names
        nColumns = 6
        '''delete previous action icons'''
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
            self.actionGrid.addLayout(actIcon, int(index/nColumns), 
                                  index%nColumns)
            self.actionIcons[index] = actIcon
            if (index == state.selected_action):
                actIcon.selected = True
                actIcon.updateView()
        if (state.action_str != ""):
            act = ActionReference.from_string(state.action_str)#state['action_str'])
            #self.l_model.clear()
            self.disp_action(act)

    def get_frame_type(self, fr_type):
        if (fr_type > 1):
            rospy.logwarn("Invalid frame type @ save_pose -> get_frame_type: " + str(fr_type))
        return ["Go to pose", "Maneuver"][fr_type]

    def n_actions(self):
        return len(self.actionIcons.keys())

    def arm_step_pressed(self, step_index):
        rospy.loginfo(step_index)
        rospy.loginfo(self.selectedStepUid)
        if step_index != self.selectedStepUid:
            self.selectedStepUid = step_index
            gui_cmd = GuiCommand(GuiCommand.SELECT_ACTION_STEP, step_index)
        else:
            self.selectedStepUid = -1
            gui_cmd = GuiCommand(GuiCommand.DESELECT_ACTION_STEP, step_index)
        self.gui_cmd_publisher.publish(gui_cmd)

    def action_pressed(self, actionIndex, isPublish=True):
        # if actionIndex == -1:
        #     self._set_enabled_widgets_in_layout(self.stepsButtonGrid, False)
        #     self._set_enabled_widgets_in_layout(self.stepsButtonGrid2, False)
        #     self._set_enabled_widgets_in_layout(self.prev_next_buttons, False)
        # else:
        #     self._set_enabled_widgets_in_layout(self.stepsButtonGrid, True)
        #     self._set_enabled_widgets_in_layout(self.stepsButtonGrid2, True)
        #     self._set_enabled_widgets_in_layout(self.prev_next_buttons, True)
        # for i in range(len(self.actionIcons.keys())):
        #     key = self.actionIcons.keys()[i]
        #     if key == actionIndex:
        #          self.actionIcons[key].selected = True
        #     else:
        #          self.actionIcons[key].selected = False
        #     self.actionIcons[key].updateView()
        # self.currentAction = actionIndex
        # self.stepsBox.setTitle('Steps for Action ' + str(self.currentAction+1))
        if isPublish:
            gui_cmd = GuiCommand(GuiCommand.SWITCH_TO_ACTION, actionIndex)
            self.gui_cmd_publisher.publish(gui_cmd)
        
    def command_cb(self):
        clickedButtonName = self._widget.sender().text()
        for key in self.commands.keys():
            if (self.commands[key] == clickedButtonName):
                qWarning('Sending speech command: '+ key)
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
