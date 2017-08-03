# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Fraunhofer nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from datetime import datetime
from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import loadUi
import getpass
import os
import socket
import time
import uuid
import xmlrpclib

from multimaster_msgs_fkie.msg import MasterState
import rospy

from master_discovery_fkie.common import resolve_url
import gui_resources
import node_manager_fkie as nm

from .capability_table import CapabilityTable
from .common import masteruri_from_ros, package_name
from .detailed_msg_box import WarningMessageBox
from .discovery_listener import MasterListService, MasterStateTopic, MasterStatisticTopic, OwnMasterMonitoring
from .launch_config import LaunchConfig  # , LaunchConfigException
from .launch_files_widget import LaunchFilesWidget
from .log_widget import LogWidget
from .master_list_model import MasterModel, MasterSyncItem
from .master_view_proxy import MasterViewProxy
from .menu_rqt import MenuRqt
from .network_discovery_dialog import NetworkDiscoveryDialog
from .parameter_dialog import ParameterDialog
from .progress_queue import ProgressQueue  # , ProgressThread
from .screen_handler import ScreenHandler
from .select_dialog import SelectDialog
from .settings_widget import SettingsWidget
from .sync_dialog import SyncDialog
from .update_handler import UpdateHandler
from .xml_editor import XmlEditor


#from python_qt_binding import QtUiTools
import roslib; roslib.load_manifest('node_manager_fkie')
try:
  from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
  DIAGNOSTICS_AVAILABLE = True
except:
  import sys
  print >> sys.stderr, "Cannot import 'diagnostic_msgs', feature disabled."
  DIAGNOSTICS_AVAILABLE = False



class MainWindow(QtGui.QMainWindow):
  '''
  The class to create the main window of the application.
  '''
  DELAYED_NEXT_REQ_ON_ERR = 5.0

  if DIAGNOSTICS_AVAILABLE:
    diagnostics_signal  = QtCore.Signal(DiagnosticStatus)
  '''@ivar: the signal is emitted if a message on topic nm_notifier was
  reiceved (DiagnosticStatus)'''

  def __init__(self, files=[], restricted_to_one_master=False, parent=None):
    '''
    Creates the window, connects the signals and init the class.
    '''
    QtGui.QMainWindow.__init__(self)
    self.default_load_launch = os.path.abspath(resolve_url(files[0])) if files else ''
    restricted_to_one_master = False
    self._finished = False
    self._history_selected_robot = ''
    self.__icons = {'empty' : (QtGui.QIcon(), ''),
                    'default_pc' : (QtGui.QIcon(':/icons/crystal_clear_miscellaneous.png'), ':/icons/crystal_clear_miscellaneous.png'),
                    'log_warning' : (QtGui.QIcon(':/icons/crystal_clear_warning.png'), ':/icons/crystal_clear_warning.png')
                    } # (masnter name : (QIcon, path))
    self.__current_icon = None
    self.__current_master_label_name = None
    self._changed_files = dict()
    self._changed_binaries = dict()
    self._changed_files_param = dict()
    self._syncs_to_start = []  # hostnames
    #self.setAttribute(QtCore.Qt.WA_AlwaysShowToolTips, True)
    # setup main window frame
    self.setObjectName('MainWindow')
#    self = mainWindow = QtGui.QMainWindow()
#    self = mainWindow = loader.load(":/forms/MainWindow.ui")
    ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MainWindow.ui')
    #/home/tiderko/ros/src/multimaster_fkie/node_manager_fkie/src/node_manager_fkie/
    loadUi(ui_file, self)
    self.setObjectName('MainUI')
    self.user_frame.setVisible(False)
    self._add_user_to_combo(getpass.getuser())
    self.userComboBox.editTextChanged.connect(self.on_user_changed)
    self.masterInfoFrame.setEnabled(False)
    self.infoButton.clicked.connect(self.on_info_clicked)
    self.setTimeButton.clicked.connect(self.on_set_time_clicked)
    self.refreshHostButton.clicked.connect(self.on_refresh_master_clicked)
    self.masterLogButton.clicked.connect(self.on_master_log_clicked)
    self.runButton.clicked.connect(self.on_run_node_clicked)
    self.syncButton.released.connect(self.on_sync_dialog_released)

    menu_rqt = MenuRqt(self.rqtButton)
    menu_rqt.start_rqt_plugin_signal.connect(self.on_rqt_plugin_start)

    # setup settings widget
    self.settings_dock = SettingsWidget(self)
    self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, self.settings_dock)
    # setup logger widget
    self.log_dock = LogWidget(self)
    self.log_dock.added_signal.connect(self._on_log_added)
    self.log_dock.cleared_signal.connect(self._on_log_cleared)
    self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, self.log_dock)
    self.logButton.clicked.connect(self._on_log_button_clicked)
    # setup the launch files view
    self.launch_dock = LaunchFilesWidget(self)
    self.launch_dock.load_signal.connect(self.on_load_launch_file)
    self.launch_dock.load_as_default_signal.connect(self.on_load_launch_as_default)
    self.launch_dock.edit_signal.connect(self.on_launch_edit)
    self.launch_dock.transfer_signal.connect(self.on_launch_transfer)
    self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, self.launch_dock)

    self.mIcon = QtGui.QIcon(":/icons/crystal_clear_prop_run.png")
    self.setWindowIcon(self.mIcon)
    self.setWindowTitle("Node Manager")
#    self.setCentralWidget(mainWindow)

    # init the stack layout which contains the information about different ros master
    self.stackedLayout = QtGui.QStackedLayout()
    self.stackedLayout.setObjectName('stackedLayout')
    emptyWidget = QtGui.QWidget()
    emptyWidget.setObjectName('emptyWidget')
    self.stackedLayout.addWidget(emptyWidget)
    self.tabWidget.currentChanged.connect(self.on_currentChanged_tab)
    self.tabLayout = QtGui.QVBoxLayout(self.tabPlace)
    self.tabLayout.setObjectName("tabLayout")
    self.tabLayout.setContentsMargins(0, 0, 0, 0)
    self.tabLayout.addLayout(self.stackedLayout)

    # initialize the progress queue
    self._progress_queue = ProgressQueue(self.progressFrame, self.progressBar, self.progressCancelButton)
    self._progress_queue_sync = ProgressQueue(self.progressFrame_sync, self.progressBar_sync, self.progressCancelButton_sync)

    # initialize the view for the discovered ROS master
    self.master_model = MasterModel(self.getMasteruri())
    self.master_model.sync_start.connect(self.on_sync_start)
    self.master_model.sync_stop.connect(self.on_sync_stop)
    self.masterTableView.setModel(self.master_model)
    self.master_model.parent_view = self.masterTableView
#    self.masterTableView.setAlternatingRowColors(True)
#    self.masterTableView.clicked.connect(self.on_master_table_clicked)
#    self.masterTableView.pressed.connect(self.on_master_table_pressed)
    self.masterTableView.activated.connect(self.on_master_table_activated)
    sm = self.masterTableView.selectionModel()
    sm.currentRowChanged.connect(self.on_masterTableView_selection_changed)
    for i, (_, width) in enumerate(MasterModel.header):#_:=name
      self.masterTableView.setColumnWidth(i, width)
    self.refreshAllButton.clicked.connect(self.on_all_master_refresh_clicked)
    self.discoveryButton.clicked.connect(self.on_discover_network_clicked)
    self.startRobotButton.clicked.connect(self.on_start_robot_clicked)

    # stores the widget to a 
    self.masters = dict() # masteruri : MasterViewProxy
    self.currentMaster = None # MasterViewProxy
    self._close_on_exit = True

    nm.file_watcher().config_changed.connect(self.on_configfile_changed)
    nm.file_watcher().binary_changed.connect(self.on_binaryfile_changed)
    nm.file_watcher_param().config_changed.connect(self.on_configparamfile_changed)
    self.__in_question = set()

    ############################################################################
    self.capabilitiesTable = CapabilityTable(self.capabilities_tab)
    self.capabilitiesTable.setObjectName("capabilitiesTable")
    self.capabilitiesTable.start_nodes_signal.connect(self.on_start_nodes)
    self.capabilitiesTable.stop_nodes_signal.connect(self.on_stop_nodes)
    self.capabilitiesTable.description_requested_signal.connect(self.on_description_update_cap)
    self.capabilities_tab.layout().addWidget(self.capabilitiesTable)

    self.descriptionTextEdit.setOpenLinks(False)
    self.descriptionTextEdit.anchorClicked.connect(self.on_description_anchorClicked)
    self._shortcut_copy = QtGui.QShortcut(QtGui.QKeySequence(self.tr("Ctrl+Shift+C", "copy selected description")), self.descriptionTextEdit)
    self._shortcut_copy.activated.connect(self.descriptionTextEdit.copy)

    self.tabifyDockWidget(self.launch_dock, self.descriptionDock)
    self.tabifyDockWidget(self.launch_dock, self.settings_dock)
    self.tabifyDockWidget(self.launch_dock, self.helpDock)
    self.launch_dock.raise_()
    self.helpDock.setWindowIcon(QtGui.QIcon(':icons/crystal_clear_helpcenter.png'))

    flags = self.windowFlags()
    self.setWindowFlags(flags | QtCore.Qt.WindowContextHelpButtonHint)

    if self.default_load_launch:
      if os.path.isdir(self.default_load_launch):
        self.launch_dock.launchlist_model.setPath(self.default_load_launch)
      elif os.path.isfile(self.default_load_launch):
        self.launch_dock.launchlist_model.setPath(os.path.dirname(self.default_load_launch))

    self._discover_dialog = None
    self.restricted_to_one_master = restricted_to_one_master
    if restricted_to_one_master:
      self.syncButton.setEnabled(False)
      self.refreshAllButton.setEnabled(False)
      self.discoveryButton.setEnabled(False)
      self.startRobotButton.setEnabled(False)

    self._sync_dialog = SyncDialog()

    self.editor_dialogs  = dict() # [file] = XmlEditor
    '''@ivar: stores the open XmlEditor '''

    self.simTimeLabel.setVisible(False)
    self.launchServerLabel.setVisible(False)

    # since the is_local method is threaded for host names, call it to cache the localhost
    nm.is_local("localhost")

    # set the help text
    try:
      from docutils import examples
      with file(nm.settings().HELP_FILE) as f:
        self.textBrowser.setText(examples.html_body(unicode(f.read())))
    except:
      import traceback
      msg = "Error while generate help: %s"%traceback.format_exc(2)
      rospy.logwarn(msg)
      self.textBrowser.setText(msg)

    try:
      ScreenHandler.testScreen()
    except Exception as e:
      rospy.logerr("No SCREEN available! You can't launch nodes.")
#      WarningMessageBox(QtGui.QMessageBox.Warning, "No SCREEN", 
#                        "No SCREEN available! You can't launch nodes.",
#                        '%s'%e).exec_()

    self.imageLabel.mouseDoubleClickEvent = self.image_mouseDoubleClickEvent

    try:
      self.readSettings()
      self.launch_dock.raise_()
    except Exception as e:
      rospy.logwarn("Error while read settings: %s"%e)
    # setup the hide button, which hides the docks on left side
    docks = self._dock_widget_in(QtCore.Qt.LeftDockWidgetArea, only_visible=True)
    if not docks:
      self.hideDocksButton.toggle()
      self.on_hide_docks_toggled(True)
    self.hideDocksButton.clicked.connect(self.on_hide_docks_toggled)

    # =============================
    # Initialize the update handler
    # =============================

    # initialize the class to get the state of discovering of other ROS master
    self._update_handler = UpdateHandler()
    self._update_handler.master_info_signal.connect(self.on_master_info_retrieved)
    self._update_handler.master_errors_signal.connect(self.on_master_errors_retrieved)
    self._update_handler.timediff_signal.connect(self.on_master_timediff_retrieved)
    self._update_handler.error_signal.connect(self.on_master_info_error)

    # this monitor class is used, if no master_discovery node is running to get the state of the local ROS master
    self.own_master_monitor = OwnMasterMonitoring()
    self.own_master_monitor.init(22622)
    self.own_master_monitor.state_signal.connect(self.on_master_state_changed)
    self.own_master_monitor.err_signal.connect(self.on_master_monitor_err)

    # get the name of the service and topic of the discovery node. The name are determine by the message type  of those topics
    self.masterlist_service = masterlist_service = MasterListService()
    masterlist_service.masterlist_signal.connect(self.on_master_list_retrieved)
    masterlist_service.masterlist_err_signal.connect(self.on_master_list_err_retrieved)
    self.state_topic = MasterStateTopic()
    self.state_topic.state_signal.connect(self.on_master_state_changed)
    self.stats_topic = MasterStatisticTopic()
    self.stats_topic.stats_signal.connect(self.on_conn_stats_updated)

    # timer to update the showed update time of the ros state 
    self.master_timecheck_timer = QtCore.QTimer()
    self.master_timecheck_timer.timeout.connect(self.on_master_timecheck)
    self.master_timecheck_timer.start(1000)
    self._refresh_time = time.time()
    self._last_time_view_update = time.time()

    self._con_tries = dict()
    self._subscribe()
    if DIAGNOSTICS_AVAILABLE:
      self._sub_extended_log = rospy.Subscriber('/diagnostics_agg', DiagnosticArray, self._callback_diagnostics)

  def _dock_widget_in(self, area=QtCore.Qt.LeftDockWidgetArea, only_visible=False):
    result = []
    docks = [self.launch_dock, self.descriptionDock, self.helpDock, self.networkDock]
    for dock in docks:
      if self.dockWidgetArea(dock) == area:
        if not only_visible or (only_visible and dock.isVisibleTo(self)):
          result.append(dock)
    return result

  def _on_log_button_clicked(self):
    self.log_dock.setVisible(not self.log_dock.isVisible())

  def _on_log_added(self, info, warn, err, fatal):
    self.logButton.setEnabled(True)

  def _on_log_cleared(self):
    self.logButton.setIcon(self.__icons['log_warning'][0])
    self.logButton.setText('')
    self.logButton.setEnabled(False)

  def on_hide_docks_toggled(self, checked):
    if self.dockWidgetArea(self.launch_dock) == QtCore.Qt.LeftDockWidgetArea:
      self.launch_dock.setVisible(not checked)
    if self.dockWidgetArea(self.descriptionDock) == QtCore.Qt.LeftDockWidgetArea:
      self.descriptionDock.setVisible(not checked)
    if self.dockWidgetArea(self.helpDock) == QtCore.Qt.LeftDockWidgetArea:
      self.helpDock.setVisible(not checked)
    if self.dockWidgetArea(self.networkDock) == QtCore.Qt.LeftDockWidgetArea:
      self.networkDock.setVisible(not checked)
    if self.dockWidgetArea(self.settings_dock) == QtCore.Qt.LeftDockWidgetArea:
      self.settings_dock.setVisible(not checked)
    self.hideDocksButton.setArrowType(QtCore.Qt.RightArrow if checked else QtCore.Qt.LeftArrow)

  def on_currentChanged_tab(self, index):
    pass
#    if index == self.tabWidget.widget(0):
#      self.networkDock.show()
#      self.launch_dock.show()
#    else:
#      self.networkDock.hide()
#      self.launch_dock.hide()

  def readSettings(self):
    if nm.settings().store_geometry:
      settings = nm.settings().qsettings(nm.settings().CFG_GUI_FILE)
      self._history_selected_robot = settings.value("selected_robot", '')
      settings.beginGroup("mainwindow")
      maximized = settings.value("maximized", 'false') == 'true'
      if maximized:
        self.showMaximized()
      else:
        self.resize(settings.value("size", QtCore.QSize(1024, 720)))
        self.move(settings.value("pos", QtCore.QPoint(0, 0)))
      try:
        self.restoreState(settings.value("window_state"))
      except:
        pass
      settings.endGroup()

  def storeSetting(self):
    if nm.settings().store_geometry:
      settings = nm.settings().qsettings(nm.settings().CFG_GUI_FILE)
      settings.beginGroup("mainwindow")
      settings.setValue("size", self.size())
      settings.setValue("pos", self.pos())
      settings.setValue("maximized", self.isMaximized())
      settings.setValue("window_state", self.saveState())
      settings.endGroup()

  def closeEvent(self, event):
    # ask to close nodes on exit
    if self._close_on_exit and nm.settings().confirm_exit_when_closing:
      res = SelectDialog.getValue('Stop nodes?', "Select masters where to stop:",
                                  self.masters.keys(), False, False, '', self,
                                  select_if_single=False,
                                  checkitem1="don't show this dialog again")
      masters2stop, self._close_on_exit = res[0], res[1]
      nm.settings().confirm_exit_when_closing = not res[2]
      if self._close_on_exit:
        self._on_finish = True
        self._stop_local_master = None
        for uri in masters2stop:
          try:
            m = self.masters[uri]
            if not m is None:
              if m.is_local:
                self._stop_updating()
                self._stop_local_master = m
              m.stop_nodes_by_name(m.getRunningNodesIfLocal(), True, [rospy.get_name(), '/rosout'])
              if not m.is_local:
                m.killall_roscore()
          except Exception as e:
            rospy.logwarn("Error while stop nodes on %s: %s"%(uri, e))
        QtCore.QTimer.singleShot(200, self._test_for_finish)
      else:
        self._close_on_exit = True
      event.ignore()
    elif self._are_master_in_process():
      QtCore.QTimer.singleShot(200, self._test_for_finish)
      self.masternameLabel.setText('<span style=" font-size:14pt; font-weight:600;">%s ...closing...</span>' % self.masternameLabel.text())
      rospy.loginfo("Wait for running processes are finished...")
      event.ignore()
    else:
      try:
        self.storeSetting()
      except Exception as e:
        rospy.logwarn("Error while store settings: %s"%e)
      self.finish()
      QtGui.QMainWindow.closeEvent(self, event)

  def _are_master_in_process(self):
    for uri, m in self.masters.items():
      if m.in_process():
        return True
    return False

  def _test_for_finish(self):
    # this method test on exit for running process queues with stopping jobs
    if self._are_master_in_process():
      QtCore.QTimer.singleShot(200, self._test_for_finish)
      return
    if hasattr(self, '_stop_local_master') and self._stop_local_master is not None:
      self.finish()
      self._stop_local_master.killall_roscore()
      del self._stop_local_master
    self._close_on_exit = False
    self.close()

  def _stop_updating(self):
    if hasattr(self, "_discover_dialog") and self._discover_dialog is not None:
      self._discover_dialog.stop()
    self.masterlist_service.stop();
    self._progress_queue.stop()
    self._progress_queue_sync.stop()
    self._update_handler.stop()
    self.state_topic.stop()
    self.stats_topic.stop()
    self.own_master_monitor.stop()
    self.master_timecheck_timer.stop()
    self.launch_dock.stop()
    self.log_dock.stop()

  def finish(self):
    if not self._finished:
      self._finished = True
      print "Mainwindow finish..."
      self._stop_updating()
      for _, master in self.masters.iteritems():
        master.stop()
      print "Mainwindow finished!"

  def getMasteruri(self):
    '''
    Requests the ROS master URI from the ROS master through the RPC interface and 
    returns it. The 'materuri' attribute will be set to the requested value.
    @return: ROS master URI
    @rtype: C{str} or C{None}
    '''
    if not hasattr(self, 'materuri') or self.materuri is None:
      masteruri = masteruri_from_ros()
      master = xmlrpclib.ServerProxy(masteruri)
      _, _, self.materuri = master.getUri(rospy.get_name())#_:=code, message
      nm.is_local(nm.nameres().getHostname(self.materuri))
    return self.materuri

  def removeMaster(self, masteruri):
    '''
    Removed master with given master URI from the list.
    @param masteruri: the URI of the ROS master
    @type masteruri: C{str}
    '''
    if masteruri in self.masters:
      if not self.currentMaster is None and self.currentMaster.masteruri == masteruri:
        self.setCurrentMaster(None)
      self.masters[masteruri].stop()
      self.masters[masteruri].updateHostRequest.disconnect()
      self.masters[masteruri].host_description_updated.disconnect()
      self.masters[masteruri].capabilities_update_signal.disconnect()
      self.masters[masteruri].remove_config_signal.disconnect()
      self.masters[masteruri].description_signal.disconnect()
      self.masters[masteruri].request_xml_editor.disconnect()
      self.masters[masteruri].stop_nodes_signal.disconnect()
      self.masters[masteruri].robot_icon_updated.disconnect()
      if DIAGNOSTICS_AVAILABLE:
        self.diagnostics_signal.disconnect(self.masters[masteruri].append_diagnostic)
      self.stackedLayout.removeWidget(self.masters[masteruri])
      self.tabPlace.layout().removeWidget(self.masters[masteruri])
      for cfg in self.masters[masteruri].default_cfgs:
        self.capabilitiesTable.removeConfig(cfg)
      self.masters[masteruri].setParent(None)
      del self.masters[masteruri]

  def getMaster(self, masteruri, create_new=True):
    '''
    @return: the Widget which represents the master of given ROS master URI. If no
    Widget for given URI is available a new one will be created.
    @rtype: L{MasterViewProxy} 
    '''
    if not masteruri in self.masters:
      if not create_new:
        return None
      self.masters[masteruri] = MasterViewProxy(masteruri, self)
      self.masters[masteruri].updateHostRequest.connect(self.on_host_update_request)
      self.masters[masteruri].host_description_updated.connect(self.on_host_description_updated)
      self.masters[masteruri].capabilities_update_signal.connect(self.on_capabilities_update)
      self.masters[masteruri].remove_config_signal.connect(self.on_remove_config)
      self.masters[masteruri].description_signal.connect(self.on_description_update)
      self.masters[masteruri].request_xml_editor.connect(self.on_launch_edit)
      self.masters[masteruri].stop_nodes_signal.connect(self.on_stop_nodes)
      self.masters[masteruri].robot_icon_updated.connect(self._on_robot_icon_changed)
      if DIAGNOSTICS_AVAILABLE:
        self.diagnostics_signal.connect(self.masters[masteruri].append_diagnostic)
      self.stackedLayout.addWidget(self.masters[masteruri])
      if masteruri == self.getMasteruri():
        if self.default_load_launch:
          try:
            if os.path.isfile(self.default_load_launch):
              args = list()
              args.append('_package:=%s'%(package_name(os.path.dirname(self.default_load_launch))[0]))
              args.append('_launch_file:="%s"'%os.path.basename(self.default_load_launch))
              host = '%s'%nm.nameres().address(masteruri)
              node_name = roslib.names.SEP.join(['%s'%(nm.nameres().mastername(masteruri)),
                                    os.path.basename(self.default_load_launch).replace('.launch',''),
                                    'default_cfg'])
              self.launch_dock.progress_queue.add2queue('%s'%uuid.uuid4(),
                                             'start default config @%s'%host,
                                             nm.starter().runNodeWithoutConfig, 
                                             ('%s'%(nm.nameres().mastername(masteruri)), 'default_cfg_fkie',
                                              'default_cfg', node_name,
                                              args, masteruri, False,
                                              self.masters[masteruri].current_user))
              self.launch_dock.progress_queue.start()
          except Exception as e:
            WarningMessageBox(QtGui.QMessageBox.Warning, "Load default configuration",
                  'Load default configuration %s failed!'%self.default_load_launch,
                  '%s'%e).exec_()
    return self.masters[masteruri]

  def on_host_update_request(self, host):
    for key, value in self.masters.items():
      if nm.nameres().getHostname(key) == host and not value.master_state is None:
        self._update_handler.requestMasterInfo(value.master_state.uri, value.master_state.monitoruri)

  def on_host_description_updated(self, masteruri, host, descr):
#    self.master_model.updateDescription(nm.nameres().mastername(masteruri, host), descr)
    pass

  def on_capabilities_update(self, masteruri, address, config_node, descriptions):
    for d in descriptions:
      self.capabilitiesTable.updateCapabilities(masteruri, config_node, d)
    if not masteruri is None:
      master = self.getMaster(masteruri)
      self.capabilitiesTable.updateState(masteruri, master.master_info)

  def on_remove_config(self, cfg):
    self.capabilitiesTable.removeConfig(cfg)

  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%            Handling of local monitoring            %%%%%%%%
  #%%%%%%%%%%%%%  (Backup, if no master_discovery node is running)  %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def _subscribe(self):
    '''
    Try to subscribe to the topics of the master_discovery node. If it fails, the
    own local monitoring of the ROS master state will be enabled.
    '''
    if not self.restricted_to_one_master:
      try:
        result_1 = self.state_topic.registerByROS(self.getMasteruri(), False)
        result_2 = self.stats_topic.registerByROS(self.getMasteruri(), False)
        self.masterlist_service.retrieveMasterList(self.getMasteruri(), False)
        self._setLocalMonitoring(not result_1 or not result_2)
      except:
        pass
    else:
      self._setLocalMonitoring(True)

  def _setLocalMonitoring(self, on):
    '''
    Enables the local monitoring of the ROS master state and disables the view of
    the discoved ROS master.
    @param on: the enable / disable the local monitoring
    @type on: C{boolean}
    '''
    self.masterTableView.setEnabled(not on)
    self.refreshAllButton.setEnabled(not on)
    self.own_master_monitor.pause(not on)
    if on:
      self.masterTableView.setToolTip("use 'Start' button to enable the master discovering")
    else:
      self.masterTableView.setToolTip('')
      self.networkDock.setWindowTitle("ROS Network [disabled]")
    if on:
      # remove discovered ROS master and set the local master to selected
      for uri in self.masters.keys():
        master = self.masters[uri]
        if nm.is_local(nm.nameres().getHostname(uri)) or uri == self.getMasteruri():
          if not self._history_selected_robot or master.mastername == self._history_selected_robot:
            self.setCurrentMaster(master)
        else:
          if not master.master_state is None:
            self.master_model.removeMaster(master.master_state.name)
          self.removeMaster(uri)
#      self.masterTableView.doItemsLayout()
    else:
      try:
        # determine the ROS network ID
        mcast_group = rospy.get_param('/master_discovery/mcast_port')
        self.networkDock.setWindowTitle("ROS Network [id: %d]" % (mcast_group - 11511))
      except:
        # try to get the multicast port of master discovery from log
        port = 0
        import re
        with open(ScreenHandler.getROSLogFile(node='/master_discovery'), 'r') as mdfile:
          for line in mdfile:
            if line.find("Listen for multicast at") > -1:
              port = map(int, re.findall(r'\d+', line))[-1]
        if port > 0:
          self.networkDock.setWindowTitle("ROS Network [id: %d]" % (port - 11511))
        else:
          self.networkDock.setWindowTitle("ROS Network")

  def on_master_list_err_retrieved(self, masteruri, error):
    '''
    The callback method connected to the signal, which is emitted on an error 
    while call the service to determine the discovered ROS master. On the error
    the local monitoring will be enabled.
    '''
    self._setLocalMonitoring(True)

  def hasDiscoveryService(self, minfo):
    '''
    Test whether the new retrieved MasterInfo contains the master_discovery node.
    This is identified by a name of the contained 'list_masters' service.
    @param minfo: the ROS master Info
    @type minfo: L{master_discovery_fkie.MasterInfo}
    '''
    # use no discovery services, if roscore is running on a remote host
    if self.restricted_to_one_master:
      return False
    for service in minfo.services.keys():
      if service.endswith('list_masters'):
        return True
    return False


  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%   Handling of received ROS master state messages   %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_master_list_retrieved(self, masteruri, servic_name, master_list):
    '''
    Handle the retrieved list with ROS master.
      1. update the ROS Network view
    @param master_list: a list with ROS masters
    @type master_list: C{[L{master_discovery_fkie.msg.MasterState}]}
    '''
    self._setLocalMonitoring(False)
    self._con_tries[masteruri] = 0
    # remove ROS master which are not in the new list
    new_uris = [m.uri for m in master_list if not m.uri is None]
    for uri in self.masters.keys():
      if uri not in new_uris:
        master = self.masters[uri]
        if not (nm.is_local(nm.nameres().getHostname(uri)) or uri == self.getMasteruri()):
          if not master.master_state is None:
            self.master_model.removeMaster(master.master_state.name)
          self.removeMaster(uri)
    # add or update master
    for m in master_list:
      if not m.uri is None:
        host = nm.nameres().getHostname(m.uri)
        nm.nameres().add_master_entry(m.uri, m.name, host)
        m.name = nm.nameres().mastername(m.uri)
        master = self.getMaster(m.uri)
        master.master_state = m
        master.force_next_update()
        self._assigne_icon(m.name)
        self.master_model.updateMaster(m)
        self._update_handler.requestMasterInfo(m.uri, m.monitoruri)

  def on_master_state_changed(self, msg):
    '''
    Handle the received master state message.
      1. update the ROS Network view
      2. enable local master monitoring, if all masters are removed (the local master too)
    @param msg: the ROS message with new master state
    @type msg: L{master_discovery_fkie.msg.MasterState}
    '''
    #'print "*on_master_state_changed"
    # do not update while closing
    if hasattr(self, "_on_finish"):
      rospy.logdebug("ignore changes on %s, because currently on closing...", msg.master.uri)
      return;
    host = nm.nameres().getHostname(msg.master.uri)
    if msg.state == MasterState.STATE_CHANGED:
      nm.nameres().add_master_entry(msg.master.uri, msg.master.name, host)
      msg.master.name = nm.nameres().mastername(msg.master.uri)
      self.getMaster(msg.master.uri).master_state = msg.master
      self._assigne_icon(msg.master.name)
      self.master_model.updateMaster(msg.master)
      if nm.settings().autoupdate:
        self._update_handler.requestMasterInfo(msg.master.uri, msg.master.monitoruri)
      else:
        rospy.loginfo("Autoupdate disabled, the data will not be updated for %s"%msg.master.uri)
    if msg.state == MasterState.STATE_NEW:
      # if new master with uri of the local master is received update the master list 
      if msg.master.uri == self.getMasteruri():
        self.masterlist_service.retrieveMasterList(msg.master.uri, False)
      nm.nameres().add_master_entry(msg.master.uri, msg.master.name, host)
      msg.master.name = nm.nameres().mastername(msg.master.uri)
      self.getMaster(msg.master.uri).master_state = msg.master
      self._assigne_icon(msg.master.name)
      self.master_model.updateMaster(msg.master)
      if nm.settings().autoupdate:
        self._update_handler.requestMasterInfo(msg.master.uri, msg.master.monitoruri)
      else:
        rospy.loginfo("Autoupdate disabled, the data will not be updated for %s"%msg.master.uri)
    if msg.state == MasterState.STATE_REMOVED:
      if msg.master.uri == self.getMasteruri():
        # switch to locale monitoring, if the local master discovering was removed
        self._setLocalMonitoring(True)
      else:
        nm.nameres().remove_master_entry(msg.master.uri)
        self.master_model.removeMaster(msg.master.name)
        self.removeMaster(msg.master.uri)
    # start master_sync, if it was selected in the start dialog to start with master_dsicovery
    if self._syncs_to_start:
      if msg.state in [MasterState.STATE_NEW, MasterState.STATE_CHANGED]:
        # we don't know which name for host was used to start master discovery
        if host in self._syncs_to_start:
          self.on_sync_start(msg.master.uri)
          self._syncs_to_start.remove(host)
        elif msg.master.name in self._syncs_to_start:
          self.on_sync_start(msg.master.uri)
          self._syncs_to_start.remove(msg.master.name)
        else:
          addresses = nm.nameres().addresses(msg.master.uri)
          for address in addresses:
            if address in self._syncs_to_start:
              self.on_sync_start(msg.master.uri)
              self._syncs_to_start.remove(address)
#      if len(self.masters) == 0:
#        self._setLocalMonitoring(True)
    #'print "**on_master_state_changed"

  def _assigne_icon(self, name, path=None):
    '''
    Sets the new icon to the given robot. If the path is `None` set search for
    .png file with robot name.
    :param name: robot name
    :type name: str
    :param path: path of the icon (Default: None)
    :type path: str
    '''
    icon_path = path if path else nm.settings().robot_image_file(name)
    if not self.__icons.has_key(name) or self.__icons[name][1] != path:
      if QtCore.QFile.exists(icon_path):
        self.__icons[name] = (QtGui.QIcon(icon_path), icon_path)
      elif self.__icons.has_key(name):
        del self.__icons[name]

  def on_master_monitor_err(self, msg):
    self._con_tries[self.getMasteruri()] += 1

  def on_master_info_retrieved(self, minfo):
    '''
    Integrate the received master info.
    @param minfo: the ROS master Info
    @type minfo: L{master_discovery_fkie.MasterInfo}
    '''
    if hasattr(self, "_on_finish"):
      rospy.logdebug("ignore changes on %s, because currently on closing...", minfo.master.uri)
      return;
    rospy.logdebug("MASTERINFO from %s (%s) received", minfo.mastername, minfo.masteruri)
    self._con_tries[minfo.masteruri] = 0
#    cputimes_m = os.times()
#    cputime_init_m = cputimes_m[0] + cputimes_m[1]
    if minfo.masteruri in self.masters:
      for _, master in self.masters.items():#_:=uri
        try:
          # check for running discovery service
          new_info = master.master_info is None or master.master_info.timestamp < minfo.timestamp
#          cputimes = os.times()
#          cputime_init = cputimes[0] + cputimes[1]
          master.master_info = minfo
#          cputimes = os.times()
#          cputime = cputimes[0] + cputimes[1] - cputime_init
#          print master.master_state.name, cputime
          if not master.master_info is None:
            if self._history_selected_robot == minfo.mastername and self._history_selected_robot == master.mastername and self.currentMaster != master:
              if not self.currentMaster is None and not self.currentMaster.is_local:
                self.setCurrentMaster(master)
            elif nm.is_local(nm.nameres().getHostname(master.master_info.masteruri)) or self.restricted_to_one_master:
              if new_info:
                has_discovery_service = self.hasDiscoveryService(minfo)
                if (not self.own_master_monitor.isPaused() or not self.masterTableView.isEnabled()) and has_discovery_service:
                  self._subscribe()
              if self.currentMaster is None and (not self._history_selected_robot or self._history_selected_robot == minfo.mastername):
                self.setCurrentMaster(master)

            # update the list view, whether master is synchronized or not
            if master.master_info.masteruri == minfo.masteruri:
              self.master_model.setChecked(master.master_state.name, not minfo.getNodeEndsWith('master_sync') is None)
          self.capabilitiesTable.updateState(minfo.masteruri, minfo)
        except Exception, e:
          rospy.logwarn("Error while process received master info from %s: %s", minfo.masteruri, str(e))
      # update the duplicate nodes
      self.updateDuplicateNodes()
      # update the buttons, whether master is synchronized or not
      if not self.currentMaster is None and not self.currentMaster.master_info is None and not self.restricted_to_one_master:
        self.syncButton.setEnabled(True)
        self.syncButton.setChecked(not self.currentMaster.master_info.getNodeEndsWith('master_sync') is None)
    else:
      self.masterlist_service.retrieveMasterList(minfo.masteruri, False)
#    cputimes_m = os.times()
#    cputime_m = cputimes_m[0] + cputimes_m[1] - cputime_init_m
#    print "ALL:", cputime_m

  def on_master_errors_retrieved(self, masteruri, error_list):
    self.master_model.updateMasterErrors(nm.nameres().mastername(masteruri), error_list)

  def on_master_timediff_retrieved(self, masteruri, timediff):
    self.master_model.updateTimeDiff(nm.nameres().mastername(masteruri), timediff)

  def on_master_info_error(self, masteruri, error):
    if not self._con_tries.has_key(masteruri):
      self._con_tries[masteruri] = 0
    self._con_tries[masteruri] += 1
    if masteruri == self.getMasteruri():
      rospy.logwarn("Error while connect to local master_discovery %s: %s", masteruri, error)
      # switch to local monitoring after 3 timeouts
      if self._con_tries[masteruri] > 2:
        self._setLocalMonitoring(True)
    master = self.getMaster(masteruri, False)
    if master and master.master_state is not None:
      self._update_handler.requestMasterInfo(master.master_state.uri, master.master_state.monitoruri, self.DELAYED_NEXT_REQ_ON_ERR)

  def on_conn_stats_updated(self, stats):
    '''
    Handle the retrieved connection statistics.
      1. update the ROS Network view
    @param stats: a list with connection statistics
    @type stats: C{[L{master_discovery_fkie.msg.LinkState}]}
    '''
    #'print "+on_conn_stats_updated"
    for stat in stats.links:
      self.master_model.updateMasterStat(stat.destination, stat.quality)
    #'print "++on_conn_stats_updated"


  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Handling of master info frame         %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_info_clicked(self):
    text = ''.join(['<dl>'])
    text = ''.join([text, '<dt><b>Maintainer</b>: ', 'Alexander Tiderko ',  '<font color=gray>alexander.tiderko@gmail.com</font>', '</dt>'])
    text = ''.join([text, '<dt><b>Author</b>: ', 'Alexander Tiderko, Timo Roehling', '</dt>'])
    text = ''.join([text, '<dt><b>License</b>: ', 'BSD, some icons are licensed under the GNU Lesser General Public License (LGPL) or Creative Commons Attribution-Noncommercial 3.0 License', '</dt>'])
    text = ''.join([text, '</dl>'])
    text = ''.join([text, '<dt><b>Version</b>: ', nm.__version__, ' (', nm.__date__,')', '</dt>'])
    QtGui.QMessageBox.about(self, 'About Node Manager', text)

  def on_master_log_clicked(self):
    '''
    Tries to get the log of master_discovery node on the machine requested by a dialog.
    '''
    # get the history list
    user_list = [self.userComboBox.itemText(i) for i in reversed(range(self.userComboBox.count()))]
    user_list.insert(0, 'last used')
    params = {'Host' : ('string', 'localhost'),
              'Show master discovery log' : ('bool', True),
              'Show master sync log' : ('bool', False),
              'Username' : ('string', user_list),
              'Only screen log' : ('bool', True),
              #'Optional Parameter' : ('list', params_optional)
              }
    dia = ParameterDialog(params, sidebar_var='Host')
    dia.setFilterVisible(False)
    dia.setWindowTitle('Show log')
    dia.resize(450,150)
    dia.setFocusField('Host')
    if dia.exec_():
      try:
        params = dia.getKeywords()
        hostnames = params['Host'] if isinstance(params['Host'], list) else [params['Host']]
        log_master_discovery = params['Show master discovery log']
        log_master_sync = params['Show master sync log']
        username = params['Username']
        screen_only = params['Only screen log']
        for hostname in hostnames:
          try:
            usr = username
            if username == 'last used':
              usr = nm.settings().host_user(hostname)
            if log_master_discovery:
              self._progress_queue.add2queue(str(uuid.uuid4()),
                                             '%s: show log of master discovery'%hostname,
                                           nm.starter().openLog,
                                           ('/master_discovery', hostname, usr, screen_only))
            if log_master_sync:
              self._progress_queue.add2queue(str(uuid.uuid4()),
                                             '%s: show log of master sync'%hostname,
                                           nm.starter().openLog,
                                           ('/master_sync', hostname, usr, screen_only))
          except (Exception, nm.StartException) as err:
            import traceback
            print traceback.format_exc(1)
            rospy.logwarn("Error while show LOG for master_discovery %s: %s"%(str(hostname), err))
            WarningMessageBox(QtGui.QMessageBox.Warning, "Show log error",
                              'Error while show log of master_discovery',
                              '%s'%err).exec_()
          self._progress_queue.start()
      except Exception as err:
        WarningMessageBox(QtGui.QMessageBox.Warning, "Show log error",
                          'Error while parse parameter',
                          '%s'%err).exec_()

  def on_set_time_clicked(self):
    if not self.currentMaster is None:# and not self.currentMaster.is_local:
      try:
        rospy.loginfo("Set remote host time to local time: %s"%self.currentMaster.master_state.uri)
        socket.setdefaulttimeout(10)
        p = xmlrpclib.ServerProxy(self.currentMaster.master_state.monitoruri)
        uri, success, newtime, errormsg = p.setTime(time.time())
        if not success:
          if errormsg.find('password') > -1:
            errormsg += "\nPlease modify /etc/sudoers and add user privilege, e.g:"
            errormsg += "\n%s  ALL=NOPASSWD: /bin/date"%self.currentMaster.current_user
            errormsg += "\n!!!needed to be at the very end of file, don't forget a new line at the end!!!"
            errormsg += "\n\nBe aware, it does not replace the time synchronization!"
            errormsg += "\nIt sets approximate time without undue delays on communication layer."
          WarningMessageBox(QtGui.QMessageBox.Warning, "Time set error",
                          'Error while set time on %s'%uri,
                          '%s'%errormsg).exec_()
        else:
          timediff = time.time() - newtime
          rospy.loginfo("  New time difference to %s is approx.: %.3fs"%(self.currentMaster.master_state.uri, timediff))
          self.on_master_timediff_retrieved(self.currentMaster.master_state.uri, timediff)
        #'print "STOP stop finished", node
      except Exception as e:
#            import traceback
#            formatted_lines = traceback.format_exc(1).splitlines()
        errormsg = '%s'%e
        if errormsg.find('setTime') > -1:
            errormsg += "\nUpdate remote multimaster_fkie!"
        rospy.logwarn("Error while set time on %s: %s"%(self.currentMaster.master_state.uri, errormsg))
        WarningMessageBox(QtGui.QMessageBox.Warning, "Time sync error",
                        'Error while set time on %s'%self.currentMaster.master_state.uri,
                        '%s'%errormsg).exec_()
      finally:
        socket.setdefaulttimeout(None)

  def on_refresh_master_clicked(self):
    if not self.currentMaster is None:
      rospy.loginfo("Request an update from %s", str(self.currentMaster.master_state.monitoruri))
      if not self.currentMaster.master_info is None:
        check_ts = self.currentMaster.master_info.check_ts
        self.currentMaster.master_info.timestamp = self.currentMaster.master_info.timestamp - 1.0
        self.currentMaster.master_info.check_ts = check_ts
      if not self.currentMaster.master_state is None:
        self._update_handler.requestMasterInfo(self.currentMaster.master_state.uri, self.currentMaster.master_state.monitoruri)
      self.currentMaster.force_next_update()
#      self.currentMaster.remove_all_def_configs()

  def on_run_node_clicked(self):
    '''
    Open a dialog to run a ROS node without a configuration
    '''
    from run_dialog import RunDialog
    if not self.currentMaster is None:
      dia = RunDialog(nm.nameres().getHostname(self.currentMaster.masteruri), self.currentMaster.masteruri)
      if dia.exec_():
        params = dia.run_params()
        if params:
          params = params + (False, self.currentMaster.current_user,) # autorequest must be false
        try:
          self._progress_queue.add2queue(str(uuid.uuid4()), 
                                         'run `%s` on %s'%(params[2], params[0]), 
                                         nm.starter().runNodeWithoutConfig, 
                                         params)
          self._progress_queue.start()
        except (Exception, nm.StartException), e:
          rospy.logwarn("Error while run `%s` on %s: %s", params[2], params[0], str(e))
          WarningMessageBox(QtGui.QMessageBox.Warning, "Run error", 
                            'Error while run node %s [%s]'%(params[2], params[1]),
                            str(e)).exec_()

  def on_rqt_plugin_start(self, name, plugin):
    if not self.currentMaster is None:
      try:
        args = []
        tool = 'rqt_gui'
        prefix = 'rqt'
        if name == 'RViz':
          prefix = 'rviz'
          tool = 'rviz'
        if plugin:
          args = ['-s', plugin]
        node_name = '%s_%s_%s' % (prefix, name.lower().replace(' ', '_'),
                                  self.currentMaster.master_state.name.replace('-', '_'))
        self.currentMaster._progress_queue.add2queue(str(uuid.uuid4()),
                                       'start %s' % name,
                                       nm.starter().runNodeWithoutConfig,
                                       ('localhost', tool, tool,
                                        node_name, args, 
                                        '%s'%self.currentMaster.master_state.uri, 
                                        False))
      except (Exception, nm.StartException), e:
        import traceback
        print traceback.format_exc(1)
        rospy.logwarn("Error while start %s: %s"%(name, e))
        WarningMessageBox(QtGui.QMessageBox.Warning, "Start error",
                          'Error while start %s'%name,
                          '%s'%e).exec_()
      self.currentMaster._progress_queue.start()

  def on_sync_dialog_released(self, released=False, masteruri=None, external_call=False):
    self.syncButton.setEnabled(False)
    master = self.currentMaster
    sync_node = None
    if not masteruri is None:
      master = self.getMaster(masteruri, False)
    if master is not None and master.master_info is not None:
      sync_node = master.master_info.getNodeEndsWith('master_sync')
    if master is not None and (sync_node is None or external_call):
      self._sync_dialog.resize(350,190)
      if self._sync_dialog.exec_():
        try:
          host = nm.nameres().getHostname(master.masteruri)
          if not self._sync_dialog.interface_filename is None:
            # copy the interface file to remote machine
            self._progress_queue_sync.add2queue(str(uuid.uuid4()),
                                           'Transfer sync interface %s'%host,
                                           nm.starter().transfer_files,
                                           ("%s"%host, self._sync_dialog.interface_filename, False, master.current_user))
          self._progress_queue_sync.add2queue(str(uuid.uuid4()),
                                         'Start sync on %s'%host,
                                         nm.starter().runNodeWithoutConfig,
                                         ("%s"%host, 'master_sync_fkie', 'master_sync', 'master_sync', self._sync_dialog.sync_args, "%s"%master.masteruri, False, master.current_user))
          self._progress_queue_sync.start()
        except:
          import traceback
          WarningMessageBox(QtGui.QMessageBox.Warning, "Start sync error",
                            "Error while start sync node",
                            str(traceback.format_exc(1))).exec_()
      else:
        self.syncButton.setChecked(False)
    elif sync_node is not None:
        master.stop_nodes([sync_node])
    self.syncButton.setEnabled(True)

  def on_sync_start(self, masteruri=None):
    '''
    Enable or disable the synchronization of the master cores
    '''
    key_mod = QtGui.QApplication.keyboardModifiers()
    if (key_mod & QtCore.Qt.ShiftModifier or key_mod & QtCore.Qt.ControlModifier):
      self.on_sync_dialog_released(masteruri=masteruri, external_call=True)
#       if not master.master_info is None:
#         node = master.master_info.getNodeEndsWith('master_sync')
#         self.syncButton.setChecked(not node is None)
    else:
      self.syncButton.setEnabled(False)
      master = self.currentMaster
      if masteruri is not None:
        master = self.getMaster(masteruri, False)
      if master is not None:
        # ask the user to start the master_sync with loaded launch file
        if not master.master_info is None:
          node = master.getNode('/master_sync')
          if node and node[0].has_configs():
            def_cfg_info = '\nNote: default_cfg parameter will be changed!' if node[0].has_default_cfgs(node[0].cfgs) else ''
            ret = QtGui.QMessageBox.question(self, 'Start synchronization','Start the synchronization using loaded configuration?\n\n `No` starts the master_sync with default parameter.%s'%def_cfg_info, QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)
            if ret == QtGui.QMessageBox.Yes:
              master.start_nodes([node[0]])
              return

        # start the master sync with default settings
        default_sync_args = ["_interface_url:='.'",
                             '_sync_topics_on_demand:=False',
                             '_ignore_hosts:=[]', '_sync_hosts:=[]',
                             '_ignore_nodes:=[]', '_sync_nodes:=[]',
                             '_ignore_topics:=[]', '_sync_topics:=[]',
                             '_ignore_services:=[]', '_sync_services:=[]',
                             '_sync_remote_nodes:=False']
        try:
          host = nm.nameres().getHostname(master.masteruri)
          self._progress_queue_sync.add2queue(str(uuid.uuid4()), 
                                         'start sync on '+str(host), 
                                         nm.starter().runNodeWithoutConfig, 
                                         (str(host), 'master_sync_fkie', 'master_sync', 'master_sync', default_sync_args, str(master.masteruri), False, master.current_user))
          self._progress_queue_sync.start()
        except:
          pass
      self.syncButton.setEnabled(True)

  def on_sync_stop(self, masteruri=None):
    master = self.currentMaster
    if masteruri is not None:
      master = self.getMaster(masteruri, False)
    if master is not None and master.master_info is not None:
      node = master.master_info.getNodeEndsWith('master_sync')
      if node is not None:
        master.stop_nodes([node])

  def on_master_timecheck(self):
    # HACK: sometimes the local monitoring will not be activated. This is the detection.
    if len(self.masters) < 2 and self.currentMaster is None:
      self._subscribe()
      return
    # update the info panel of the robot. If the node manager is not selected the updates are rarer. 
    current_time = time.time()
    if self.isActiveWindow() or current_time - self._last_time_view_update > 5:
      self._last_time_view_update = current_time
      if not self.currentMaster is None and not self.currentMaster.master_state is None:
        master = self.getMaster(self.currentMaster.master_state.uri)
        name = master.master_state.name
        masteruri = master.master_state.uri
        if self.restricted_to_one_master:
          name = ''.join([name, ' <span style=" color:red;">(restricted)</span>'])
          if not self.masternameLabel.toolTip():
            self.masternameLabel.setToolTip('The multicore options are disabled, because the roscore is running on remote host!')
        if not master.master_info is None:
          self.showMasterName(masteruri, name, self.timestampStr(master.master_info.check_ts), master.master_state.online)
          pass
        elif not master.master_state is None:
          text = 'Try to get info!!!'
          if not nm.settings().autoupdate:
            text = 'Press F5 or click on reload to get info'
          self.showMasterName(masteruri, name, text, master.master_state.online)
      else:
        self.showMasterName('', 'No robot selected', None, False)
    if (current_time - self._refresh_time > 30.0):
      masteruri = self.getMasteruri()
      if not masteruri is None:
        master = self.getMaster(masteruri)
        if not master is None and not master.master_state is None and nm.settings().autoupdate:
          self._update_handler.requestMasterInfo(master.master_state.uri, master.master_state.monitoruri)
        self._refresh_time = current_time


  def showMasterName(self, masteruri, name, timestamp, online=True):
    '''
    Update the view of the info frame.
    '''
    con_err = ''
    try:
      tries = self._con_tries[masteruri]
      if tries > 1:
        con_err = '<span style=" color:red;">connection problems (%s tries)! </span>'%str(tries)
    except:
      pass
    if self.__current_master_label_name != name:
      self.__current_master_label_name = name
      self.masternameLabel.setText('<span style=" font-size:14pt; font-weight:600;">%s</span>'%name)
    ts = 'updated: %s'%str(timestamp) if not timestamp is None else ''
    if not nm.settings().autoupdate:
      ts = '%s<span style=" color:orange;"> AU off</span>'%ts
    self.masterInfoLabel.setText('<span style=" font-size:8pt;">%s%s</span>'%(con_err, ts))

    # load the robot image, if one exists
    if self.masternameLabel.isEnabled():
      if self.__icons.has_key(name):
        if self.__icons[name][0] != self.__current_icon:
          icon = self.__icons[name][0]
          self.__current_icon = icon
          self.imageLabel.setPixmap(icon.pixmap(self.nameFrame.size()))
          self.imageLabel.setToolTip(''.join(['<html><head></head><body><img src="', self.__icons[name][1], '" alt="', name,'"></body></html>']))
      elif self.__icons['default_pc'][0] != self.__current_icon:
        icon = self.__icons['default_pc'][0]
        self.__current_icon = icon
        self.imageLabel.setPixmap(icon.pixmap(self.nameFrame.size()))
        self.imageLabel.setToolTip('')
    # set sim_time info
    master = self.getMaster(masteruri, False)
    sim_time_enabled = self.masternameLabel.isEnabled() and not master is None and master.use_sim_time
    self.simTimeLabel.setVisible(bool(sim_time_enabled))
    launch_server_enabled = self.masternameLabel.isEnabled() and (not master is None) and master.has_launch_server()
    self.launchServerLabel.setVisible(launch_server_enabled)
    self.masternameLabel.setEnabled(online)
    self.masterInfoFrame.setEnabled((not timestamp is None))
    # update warning symbol / text
    if self.logButton.isEnabled():
      if self.logButton.text():
        self.logButton.setIcon(self.__icons['log_warning'][0])
        self.logButton.setText('')
      else:
        self.logButton.setText('%d'%self.log_dock.count())
        self.logButton.setIcon(self.__icons['empty'][0])


  def timestampStr(self, timestamp):
    dt = datetime.fromtimestamp(timestamp)
    diff = time.time()-timestamp
    diff_dt = datetime.fromtimestamp(diff)
    before = '0 sec'
    if (diff < 60):
      before = diff_dt.strftime('%S sec')
    elif (diff < 3600):
      before = diff_dt.strftime('%M:%S min')
    elif (diff < 86400):
      before = diff_dt.strftime('%H:%M:%S std')
    else:
      before = diff_dt.strftime('%d Day(s) %H:%M:%S')
    return '%s (%s)'%(dt.strftime('%H:%M:%S'), before)

  def updateDuplicateNodes(self):
    # update the duplicate nodes
    running_nodes = dict()
    for _, m in self.masters.items():
      if not m.master_state is None and m.master_state.online:
        running_nodes.update(m.getRunningNodesIfLocal())
    for _, m in self.masters.items():
      if not m.master_state is None:
        m.markNodesAsDuplicateOf(running_nodes)



  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Handling of master list view          %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_master_table_pressed(self, selected):
    pass

  def on_master_table_clicked(self, selected):
    '''
    On click on the sync item, the master_sync node will be started or stopped,
    depending on run state.
    '''
    pass
#     item = self.master_model.itemFromIndex(selected)
#     if isinstance(item, MasterSyncItem):
#       pass

  def on_master_table_activated(self, selected):
    item = self.master_model.itemFromIndex(selected)
    QtGui.QMessageBox.information(self, item.name, item.toolTip())

  def on_master_selection_changed(self, selected):
    '''
    If a master was selected, set the corresponding Widget of the stacked layout
    to the current widget and shows the state of the selected master.
    '''
#     si = self.masterTableView.selectedIndexes()
#     for index in si:
#       if index.row() == selected.row():
    item = self.master_model.itemFromIndex(selected)
    if not item is None:
      self._history_selected_robot = item.master.name
      self.setCurrentMaster(item.master.uri)
      if not self.currentMaster.master_info is None and not self.restricted_to_one_master:
        node = self.currentMaster.master_info.getNodeEndsWith('master_sync')
        self.syncButton.setEnabled(True)
        self.syncButton.setChecked(not node is None)
      else:
        self.syncButton.setEnabled(False)
      return
    self.launch_dock.raise_()

  def setCurrentMaster(self, master):
    '''
    Changes the view of the master.
    :param master: the MasterViewProxy object or masteruri
    :type master: MasterViewProxy or str
    '''
    show_user_field = False
    if isinstance(master, MasterViewProxy):
      self.currentMaster = master
      self.stackedLayout.setCurrentWidget(master)
      show_user_field = not master.is_local
      self._add_user_to_combo(self.currentMaster.current_user)
      self.userComboBox.setEditText(self.currentMaster.current_user)
    elif master is None:
      self.currentMaster = None
      self.stackedLayout.setCurrentIndex(0)
    else: # it's masteruri
      self.currentMaster = self.getMaster(master)
      if not self.currentMaster is None:
        self.stackedLayout.setCurrentWidget(self.currentMaster)
        show_user_field = not self.currentMaster.is_local
        self._add_user_to_combo(self.currentMaster.current_user)
        self.userComboBox.setEditText(self.currentMaster.current_user)
      else:
        self.stackedLayout.setCurrentIndex(0)
    self.user_frame.setVisible(show_user_field)
    self.on_master_timecheck()

  def _add_user_to_combo(self, user):
    for i in range(self.userComboBox.count()):
      if user.lower() == self.userComboBox.itemText(i).lower():
        return
    self.userComboBox.addItem(user)

  def on_user_changed(self, user):
    if not self.currentMaster is None:
      self.currentMaster.current_user = user

  def on_masterTableView_selection_changed(self, selected, deselected):
    '''
    On selection of a master list.
    '''
    if selected.isValid():
      self.on_master_selection_changed(selected)

  def on_all_master_refresh_clicked(self):
    '''
    Retrieves from the master_discovery node the list of all discovered ROS 
    master and get their current state.
    '''
    # set the timestamp of the current master info back
    for _, m in self.masters.items():
      if not m.master_info is None:
        check_ts = m.master_info.check_ts
        m.master_info.timestamp = m.master_info.timestamp - 1.0
        m.master_info.check_ts = check_ts
    self.masterlist_service.refresh(self.getMasteruri(), False)

  def on_discover_network_clicked(self):
    try:
      self._discover_dialog.raise_()
    except:
      mcast_group = rospy.get_param('/master_discovery/mcast_group', '226.0.0.0')
      self._discover_dialog = NetworkDiscoveryDialog(mcast_group, 11511, 100, self)
      self._discover_dialog.network_join_request.connect(self._join_network)
      self._discover_dialog.show()

  def on_start_robot_clicked(self):
    '''
    Tries to start the master_discovery node on the machine requested by a dialog.
    '''
    # get the history list
    user_list = [self.userComboBox.itemText(i) for i in reversed(range(self.userComboBox.count()))]
    user_list.insert(0, 'last used')
    params_optional = {'Discovery type': ('string', ['master_discovery', 'zeroconf']),
                       'ROS Master Name' : ('string', 'autodetect'),
                       'ROS Master URI' : ('string', 'ROS_MASTER_URI'),
                       'Robot hosts' : ('string', ''),
                       'Username' : ('string', user_list),
                       'MCast Group' : ('string', '226.0.0.0'),
                       'Heartbeat [Hz]' : ('float', 0.5)
                      }
    params = {'Host' : ('string', 'localhost'),
              'Network(0..99)' : ('int', '0'),
              'Start sync' : ('bool', nm.settings().start_sync_with_discovery),
              'Optional Parameter' : ('list', params_optional) }
    dia = ParameterDialog(params, sidebar_var='Host')
    dia.setFilterVisible(False)
    dia.setWindowTitle('Start discovery')
    dia.resize(450, 330)
    dia.setFocusField('Host')
    if dia.exec_():
      try:
        params = dia.getKeywords()
        hostnames = params['Host'] if isinstance(params['Host'], list) else [params['Host']]
        port = params['Network(0..99)']
        start_sync = params['Start sync']
        discovery_type = params['Optional Parameter']['Discovery type']
        mastername = 'autodetect'
        masteruri = 'ROS_MASTER_URI'
        if len(hostnames) < 2:
          mastername = params['Optional Parameter']['ROS Master Name']
          masteruri = params['Optional Parameter']['ROS Master URI']
        robot_hosts = params['Optional Parameter']['Robot hosts']
        username = params['Optional Parameter']['Username']
        mcast_group = params['Optional Parameter']['MCast Group']
        heartbeat_hz = params['Optional Parameter']['Heartbeat [Hz]']
        if robot_hosts:
          robot_hosts = robot_hosts.replace(' ', '')
          robot_hosts = robot_hosts.replace('[', '')
          robot_hosts = robot_hosts.replace(']', '')
        for hostname in hostnames:
          try:
            args = []
            if port is not None and port and int(port) < 100 and int(port) >= 0:
              args.append('_mcast_port:=%s'%(11511 + int(port)))
            else:
              args.append('_mcast_port:=%s'%(11511))
            if not mastername == 'autodetect':
              args.append('_name:=%s'%(mastername))
            args.append('_mcast_group:=%s'%mcast_group)
            args.append('_robot_hosts:=[%s]'%robot_hosts)
            args.append('_heartbeat_hz:=%s'%heartbeat_hz)
            #TODO: remove the name parameter from the ROS parameter server
            usr = username
            if username == 'last used':
              usr = nm.settings().host_user(hostname)
            muri = None if masteruri == 'ROS_MASTER_URI' else str(masteruri)
            self._progress_queue.add2queue(str(uuid.uuid4()), 
                                           'start discovering on %s'%hostname,
                                           nm.starter().runNodeWithoutConfig, 
                                           (str(hostname), 'master_discovery_fkie', str(discovery_type), str(discovery_type), args, muri, False, usr))

            # start the master sync with default settings
            if start_sync:
              if nm.is_local(hostname):
                default_sync_args = ["_interface_url:='.'",
                                     '_sync_topics_on_demand:=False',
                                     '_ignore_hosts:=[]', '_sync_hosts:=[]',
                                     '_ignore_nodes:=[]', '_sync_nodes:=[]',
                                     '_ignore_topics:=[]', '_sync_topics:=[]',
                                     '_ignore_services:=[]', '_sync_services:=[]',
                                     '_sync_remote_nodes:=False']
                self._progress_queue_sync.add2queue(str(uuid.uuid4()),
                                               'start sync on %s' % hostname,
                                               nm.starter().runNodeWithoutConfig,
                                               (str(hostname), 'master_sync_fkie', 'master_sync', 'master_sync', default_sync_args, muri, False, usr))
                self._progress_queue_sync.start()
              else:
                if hostname not in self._syncs_to_start:
                  self._syncs_to_start.append(hostname)
          except (Exception, nm.StartException) as e:
            import traceback
            print traceback.format_exc(1)
            rospy.logwarn("Error while start master_discovery for %s: %s"%(str(hostname), e))
            WarningMessageBox(QtGui.QMessageBox.Warning, "Start error", 
                              'Error while start master_discovery',
                              str(e)).exec_()
          self._progress_queue.start()
      except Exception as e:
        WarningMessageBox(QtGui.QMessageBox.Warning, "Start error", 
                          'Error while parse parameter',
                          str(e)).exec_()

  def _join_network(self, network):
    try:
      hostname = 'localhost'
      args = []
      if network < 100 and network >= 0:
        args.append(''.join(['_mcast_port:=', str(11511 + int(network))]))
      self._progress_queue.add2queue(str(uuid.uuid4()), 
                                     'start discovering on '+str(hostname), 
                                     nm.starter().runNodeWithoutConfig, 
                                     (str(hostname), 'master_discovery_fkie', 'master_discovery', 'master_discovery', args, None, False))
      self._progress_queue.start()
    except (Exception, nm.StartException), e:
      rospy.logwarn("Error while start master_discovery for %s: %s", str(hostname), str(e))
      WarningMessageBox(QtGui.QMessageBox.Warning, "Start error", 
                        'Error while start master_discovery',
                        str(e)).exec_()

  def poweroff_host(self, host):
    try:
      if nm.is_local(str(host)):
        ret = QtGui.QMessageBox.warning(self, "ROS Node Manager",
                                 "Do you really want to shutdown localhost?",
                                 QtGui.QMessageBox.Ok | QtGui.QMessageBox.Cancel)
        if ret == QtGui.QMessageBox.Cancel:
          return
      masteruris = nm.nameres().masterurisbyaddr(host)
      for masteruri in masteruris:
        master = self.getMaster(masteruri)
        master.stop_nodes_by_name(['/master_discovery'])
      self._progress_queue.add2queue(str(uuid.uuid4()),
                                     'poweroff `%s`'%host,
                                     nm.starter().poweroff,
                                     ('%s'%host,))
      self._progress_queue.start()
      self.on_description_update('Description', '')
      self.launch_dock.raise_()
    except (Exception, nm.StartException), e:
      rospy.logwarn("Error while poweroff %s: %s", host, str(e))
      WarningMessageBox(QtGui.QMessageBox.Warning, "Run error",
                        'Error while poweroff %s'%host,
                        '%s'%e).exec_()

  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%         Handling of the launch view signals        %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_load_launch_file(self, path):
    '''
    Load the launch file. A ROS master must be selected first.
    :param path: the path of the launch file.
    :type path: str
    '''
    rospy.loginfo("LOAD launch: %s"%path)
    master_proxy = self.stackedLayout.currentWidget()
    if isinstance(master_proxy, MasterViewProxy):
#      cursor = self.cursor()
#      self.setCursor(QtCore.Qt.WaitCursor)
      try:
        master_proxy.launchfiles = path
      except Exception, e:
        import traceback
        print traceback.format_exc(1)
        WarningMessageBox(QtGui.QMessageBox.Warning, "Loading launch file", path, '%s'%e).exec_()
#      self.setCursor(cursor)
    else:
      QtGui.QMessageBox.information(self, "Load of launch file",
                                    "Select a master first!", )

  def on_load_launch_as_default(self, path, host=None):
    '''
    Load the launch file as default configuration. A ROS master must be selected first.
    :param path: the path of the launch file.
    :type path: str
    :param host: The host name, where the configuration start.
    :type host: str (Default: None)
    '''
    rospy.loginfo("LOAD launch as default: %s"%path)
    master_proxy = self.stackedLayout.currentWidget()
    if isinstance(master_proxy, MasterViewProxy):
      args = list()
      args.append('_package:=%s'%(package_name(os.path.dirname(path))[0]))
      args.append('_launch_file:="%s"'%os.path.basename(path))
      try:
        # test for requerid args
        launchConfig = LaunchConfig(path)
        req_args = launchConfig.getArgs()
        if req_args:
          params = dict()
          arg_dict = launchConfig.argvToDict(req_args)
          for arg in arg_dict.keys():
            params[arg] = ('string', [arg_dict[arg]])
          inputDia = ParameterDialog(params)
          inputDia.setFilterVisible(False)
          inputDia.setWindowTitle('Enter the argv for %s'%path)
          if inputDia.exec_():
            params = inputDia.getKeywords()
            args.extend(launchConfig.resolveArgs([''.join([p, ":='", v, "'"]) for p,v in params.items() if v]))
          else:
            return
      except:
        import traceback
        rospy.logwarn('Error while load %s as default: %s'%(path, traceback.format_exc(1)))
      hostname = host if host else nm.nameres().address(master_proxy.masteruri)
      name_file_prefix = os.path.basename(path).replace('.launch','').replace(' ', '_')
      node_name = roslib.names.SEP.join([hostname,
                                         name_file_prefix,
                                         'default_cfg'])
      self.launch_dock.progress_queue.add2queue('%s'%uuid.uuid4(),
                                     'start default config %s'%hostname,
                                     nm.starter().runNodeWithoutConfig,
                                     ('%s'%hostname, 'default_cfg_fkie', 'default_cfg',
                                      node_name, args, master_proxy.masteruri, False,
                                      master_proxy.current_user))
      self.launch_dock.progress_queue.start()
    else:
      QtGui.QMessageBox.information(self, "Load of launch file",
                                    "Select a master first!", )


  def on_launch_edit(self, files, search_text='', trynr=1):
    '''
    Opens the given files in an editor. If the first file is already open, select
    the editor. If search text is given, search for the text in files an goto the
    line.
    :param file: A list with paths
    :type file: list of strings
    :param search_text: A string to search in file
    :type search_text: str
    '''
    if files:
      path = files[0]
      if self.editor_dialogs.has_key(path):
        last_path = files[-1]
        try:
          self.editor_dialogs[path].on_load_request(last_path, search_text)
          self.editor_dialogs[path].raise_()
          self.editor_dialogs[path].activateWindow()
        except:
          if trynr > 1:
            raise
          del self.editor_dialogs[path]
          self.on_launch_edit(files, search_text, 2)
      else:
        editor = XmlEditor(files, search_text, self)
        self.editor_dialogs[path] = editor
        editor.finished_signal.connect(self._editor_dialog_closed)
        editor.show()

  def _editor_dialog_closed(self, files):
    '''
    If a editor dialog is closed, remove it from the list...
    '''
    if self.editor_dialogs.has_key(files[0]):
      del self.editor_dialogs[files[0]]

  def on_launch_transfer(self, files):
    '''
    Copies the selected file to a remote host
    :param file: A list with paths
    :type file: list of strings
    '''
    if files:
      host = 'localhost'
      username = nm.settings().default_user
      if not self.currentMaster is None:
        host = nm.nameres().getHostname(self.currentMaster.masteruri)
        username = self.currentMaster.current_user
      params = {'Host' : ('string', host),
                'recursive' : ('bool', 'False'),
                'Username' : ('string', '%s'%username) }
      dia = ParameterDialog(params)
      dia.setFilterVisible(False)
      dia.setWindowTitle('Transfer file')
      dia.resize(350,120)
      dia.setFocusField('Host')
      if dia.exec_():
        try:
          params = dia.getKeywords()
          host = params['Host']
          recursive = params['recursive']
          username = params['Username']
          for path in files:
            rospy.loginfo("TRANSFER to %s@%s: %s"%(username, host, path))
            self.launch_dock.progress_queue.add2queue('%s'%uuid.uuid4(),
                                           'transfer files to %s'%host,
                                           nm.starter().transfer_files, 
                                           ('%s'%host, path, False, username))
            if recursive:
              for f in LaunchConfig.getIncludedFiles(path):
                self.launch_dock.progress_queue.add2queue(str(uuid.uuid4()),
                                               'transfer files to %s'%host,
                                               nm.starter().transfer_files,
                                               ('%s'%host, f, False, username))
          self.launch_dock.progress_queue.start()
        except Exception, e:
          WarningMessageBox(QtGui.QMessageBox.Warning, "Transfer error",
                           'Error while transfer files', '%s'%e).exec_()

  def _reload_globals_at_next_start(self, launch_file):
    if not self.currentMaster is None:
      self.currentMaster.reload_global_parameter_at_next_start(launch_file)

  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Change file detection      %%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_configfile_changed(self, changed, affected):
    '''
    Signal hander to handle the changes of a loaded configuration file
    @param changed: the changed file
    @type changed: C{str}
    @param affected: the list of tuples with masteruri and launchfile, which are affected by file change
    @type affected: list
    '''
    # create a list of launch files and masters, which are affected by the changed file
    # and are not currently in question
    if self.isActiveWindow():
      self._changed_files[changed] = affected
      self._check_for_changed_files()
    else:
      self._changed_files[changed] = affected

  def on_binaryfile_changed(self, changed, affected):
    '''
    Signal hander to handle the changes started binaries.
    @param changed: the changed file
    @type changed: str
    @param affected: list of tuples(node name, masteruri, launchfile), which are
                     affected by file change
    @type affected: list
    '''
    if self.isActiveWindow():
      self._changed_binaries[changed] = affected
      self._check_for_changed_files()
    else:
      self._changed_binaries[changed] = affected

  def _check_for_changed_files(self):
    '''
    Check the dictinary with changed files and notify the masters about changes.
    '''
    new_affected = list()
    for _, affected in self._changed_files.items():#:=changed
      for (muri, lfile) in affected:
        if not (muri, lfile) in self.__in_question:
          self.__in_question.add((muri, lfile))
          new_affected.append((muri, lfile))
    # if there are no question to reload the launch file -> ask
    if new_affected:
      choices = dict()
      for (muri, lfile) in new_affected:
        master = self.getMaster(muri)
        if not master is None:
          master.launchfile = lfile
          choices[''.join([os.path.basename(lfile), ' [', master.mastername, ']'])] = (master, lfile)
      cfgs, _ = SelectDialog.getValue('Reload configurations?',
                                   '<b>%s</b> was changed.<br>Select affected configurations to reload:'%', '.join([os.path.basename(f) for f in self._changed_files.keys()]), choices.keys(),
                                   False, True,
                                   ':/icons/crystal_clear_launch_file.png',
                                   self)
      for (muri, lfile) in new_affected:
        self.__in_question.remove((muri, lfile))
      for c in cfgs:
        choices[c][0].launchfiles = choices[c][1]
    self._changed_files.clear()

  def _check_for_changed_binaries(self):
    '''
    Check the dictinary with changed binaries and notify the masters about changes.
    '''
    new_affected = list()
    for _, affected in self._changed_binaries.items():#:=changed
      for (nname, muri, lfile) in affected:
        if not (nname, muri, lfile) in self.__in_question:
          self.__in_question.add((nname, muri, lfile))
          new_affected.append((nname, muri, lfile))
    # if there are no question to restart the nodes -> ask
    if new_affected:
      choices = dict()
      for (nname, muri, lfile) in new_affected:
        master = self.getMaster(muri)
        if not master is None:
          master_nodes = master.getNode(nname)
          if master_nodes and master_nodes[0].is_running():
            choices[nname] = (master, lfile)
          else:
            nm.file_watcher().rem_binary(nname)
      if choices:
        nodes, _ = SelectDialog.getValue('Restart nodes?',
                                     '<b>%s</b> was changed.<br>Select affected nodes to restart:'%', '.join([os.path.basename(f) for f in self._changed_binaries.keys()]), choices.keys(),
                                     False, True,
                                     '',
                                     self)
        for (nname, muri, lfile) in new_affected:
          self.__in_question.remove((nname, muri, lfile))
        for nname in nodes:
          choices[nname][0].stop_nodes_by_name([nname])
        for nname in nodes:
          choices[nname][0].start_nodes_by_name([nname], choices[nname][1], True)
    self._changed_binaries.clear()

  def on_configparamfile_changed(self, changed, affected):
    '''
    Signal handler to handle the changes of a configuration file referenced by a parameter value
    @param changed: the changed file
    @type changed: C{str}
    @param affected: the list of tuples with masteruri and launchfile, which are affected by file change
    @type affected: list
    '''
    # create a list of launch files and masters, which are affected by the changed file
    # and are not currently in question
    if self.isActiveWindow():
      self._changed_files_param[changed] = affected
      self._check_for_changed_files_param()
    else:
      self._changed_files_param[changed] = affected

  def _check_for_changed_files_param(self):
    '''
    Check the dictinary with changed files and notify about the transfer of changed file.
    '''
    new_affected = list()
    for changed, affected in self._changed_files_param.items():
      for (muri, lfile) in affected:
        if not (muri, changed) in self.__in_question:
          self.__in_question.add((muri, changed))
          new_affected.append((muri, changed))
    # if there are no question to reload the launch file -> ask
    if new_affected:
      choices = dict()
      for (muri, lfile) in new_affected:
        master = self.getMaster(muri)
        if not master is None:
          master.launchfile = lfile
          choices[''.join([os.path.basename(lfile), ' [', master.mastername, ']'])] = (master, lfile)
      cfgs, _ = SelectDialog.getValue('Transfer configurations?',
                                   'Configuration files referenced by parameter are changed.<br>Select affected configurations for copy to remote host: (don\'t forget to restart the nodes!)', 
                                   choices.keys(), False, True,
                                   ':/icons/crystal_clear_launch_file_transfer.png',
                                   self)
      for (muri, lfile) in new_affected:
        self.__in_question.remove((muri, lfile))
      for c in cfgs:
        host = '%s'%nm.nameres().getHostname(choices[c][0].masteruri)
        username = choices[c][0].current_user
        self.launch_dock.progress_queue.add2queue(str(uuid.uuid4()),
                                       'transfer files to %s'%host,
                                       nm.starter().transfer_files,
                                       (host, choices[c][1], False, username))
      self.launch_dock.progress_queue.start()
    self._changed_files_param.clear()

  def changeEvent(self, event):
    '''
    Check for changed files, if the main gui is activated.
    '''
    QtGui.QMainWindow.changeEvent(self, event)
    self._check_for_changed_files()
    self._check_for_changed_binaries()
    self._check_for_changed_files_param()

  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Capabilities handling      %%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def on_start_nodes(self, masteruri, cfg, nodes):
    if not masteruri is None:
      master = self.getMaster(masteruri)
      master.start_nodes_by_name(nodes, (cfg, ''))

  def on_stop_nodes(self, masteruri, nodes):
    if not masteruri is None:
      master = self.getMaster(masteruri)
      master.stop_nodes_by_name(nodes)

  def on_description_update(self, title, text, force=False):
    same_title = self.descriptionDock.windowTitle() == title
    valid_sender = self.sender() == self.currentMaster or not isinstance(self.sender(), MasterViewProxy)
    no_focus = not self.descriptionTextEdit.hasFocus()
    if (valid_sender) and (same_title or no_focus or self._accept_next_update):
      self._accept_next_update = False
      self.descriptionDock.setWindowTitle(title)
      self.descriptionTextEdit.setText(text)
      if text and force:  # and not (self.launch_dock.hasFocus() or self.launch_dock.xmlFileView.hasFocus()):
        self.descriptionDock.raise_()
#      else:
#        self.launch_dock.raise_()

  def on_description_update_cap(self, title, text):
    self.descriptionDock.setWindowTitle(title)
    self.descriptionTextEdit.setText(text)

  def on_description_anchorClicked(self, url):
    self._accept_next_update = True
    if url.toString().startswith('open_sync_dialog://'):
      self.on_sync_dialog_released(False, str(url.encodedPath()).replace('open_sync_dialog', 'http'), True)
    elif url.toString().startswith('show_all_screens://'):
      master = self.getMaster(str(url.encodedPath()).replace('show_all_screens', 'http'), False)
      if not master is None:
        master.on_show_all_screens()
    elif url.toString().startswith('remove_all_launch_server://'):
      master = self.getMaster(str(url.encodedPath()).replace('remove_all_launch_server', 'http'), False)
      if not master is None:
        master.on_remove_all_launch_server()
    elif url.toString().startswith('node://'):
      if not self.currentMaster is None:
        self.currentMaster.on_node_selection_changed(None, None, True, url.encodedPath())
    elif url.toString().startswith('topic://'):
      if not self.currentMaster is None:
        self.currentMaster.on_topic_selection_changed(None, None, True, url.encodedPath())
    elif url.toString().startswith('topicecho://'):
      if not self.currentMaster is None:
        self.currentMaster.show_topic_output(url.encodedPath(), False)
    elif url.toString().startswith('topichz://'):
      if not self.currentMaster is None:
        self.currentMaster.show_topic_output(url.encodedPath(), True)
    elif url.toString().startswith('topichzssh://'):
      if not self.currentMaster is None:
        self.currentMaster.show_topic_output(url.encodedPath(), True, use_ssh=True)
    elif url.toString().startswith('topicpub://'):
      if not self.currentMaster is None:
        self.currentMaster.start_publisher(url.encodedPath())
    elif url.toString().startswith('topicrepub://'):
      if not self.currentMaster is None:
        self.currentMaster.start_publisher(url.encodedPath(), True)
    elif url.toString().startswith('topicstop://'):
      if not self.currentMaster is None:
        self.currentMaster.on_topic_pub_stop_clicked(url.encodedPath())
    elif url.toString().startswith('service://'):
      if not self.currentMaster is None:
        self.currentMaster.on_service_selection_changed(None, None, True, url.encodedPath())
    elif url.toString().startswith('servicecall://'):
      if not self.currentMaster is None:
        self.currentMaster.service_call(url.encodedPath())
    elif url.toString().startswith('unregister_node://'):
      if not self.currentMaster is None:
        self.currentMaster.on_unregister_nodes()
    elif url.toString().startswith('start_node://'):
      if not self.currentMaster is None:
        self.currentMaster.on_start_clicked()
    elif url.toString().startswith('restart_node://'):
      if not self.currentMaster is None:
        self.currentMaster.on_force_start_nodes()
    elif url.toString().startswith('start_node_at_host://'):
      if not self.currentMaster is None:
        self.currentMaster.on_start_nodes_at_host()
    elif url.toString().startswith('start_node_adv://'):
      if not self.currentMaster is None:
        self.currentMaster.on_start_alt_clicked()
    elif url.toString().startswith('kill_node://'):
      if not self.currentMaster is None:
        self.currentMaster.on_kill_nodes()
    elif url.toString().startswith('kill_screen://'):
      if not self.currentMaster is None:
        self.currentMaster.on_kill_screens()
    elif url.toString().startswith('copy_log_path://'):
      if not self.currentMaster is None:
        self.currentMaster.on_log_path_copy()
    elif url.toString().startswith('launch://'):
      self.on_launch_edit([str(url.encodedPath())], '')
    elif url.toString().startswith('reload_globals://'):
      self._reload_globals_at_next_start(str(url.encodedPath()).replace('reload_globals://', ''))
    elif url.toString().startswith('poweroff://'):
      self.poweroff_host(url.encodedHost())
    else:
      QtGui.QDesktopServices.openUrl(url)

  def keyReleaseEvent(self, event):
    '''
    Defines some of shortcuts for navigation/management in launch
    list view or topics view.
    '''
    key_mod = QtGui.QApplication.keyboardModifiers()
    if not self.currentMaster is None and self.currentMaster.masterTab.nodeTreeView.hasFocus():
      if event.key() == QtCore.Qt.Key_F4 and not key_mod:
        if self.currentMaster.masterTab.editConfigButton.isEnabled():
          self.currentMaster.on_edit_config_clicked()
        elif self.currentMaster.masterTab.editRosParamButton.isEnabled():
          self.currentMaster.on_edit_rosparam_clicked()
      elif event.key() == QtCore.Qt.Key_F3  and not key_mod and self.currentMaster.masterTab.ioButton.isEnabled():
        self.currentMaster.on_io_clicked()
    QtGui.QMainWindow.keyReleaseEvent(self, event)

  def image_mouseDoubleClickEvent(self, event):
    '''
    Set the robot image
    '''
    if self.currentMaster:
      try:
        if not os.path.isdir(nm.settings().ROBOTS_DIR):
          os.makedirs(nm.settings().ROBOTS_DIR)
        (fileName, _) = QtGui.QFileDialog.getOpenFileName(self,
                                                 "Set robot image",
                                                 nm.settings().ROBOTS_DIR,
                                                 "Image files (*.bmp *.gif *.jpg *.jpeg *.png *.pbm *.xbm);;All files (*)")
        if fileName and self.__current_master_label_name:
          p = QtGui.QPixmap(fileName)
          p.save(nm.settings().robot_image_file(self.__current_master_label_name))
        if self.__icons.has_key(self.__current_master_label_name):
          del self.__icons[self.__current_master_label_name]
        self._assigne_icon(self.__current_master_label_name)
      except Exception as e:
        WarningMessageBox(QtGui.QMessageBox.Warning, "Error", 
                          ''.join(['Set robot image for ', str(self.__current_master_label_name), ' failed!']),
                          str(e)).exec_()
        rospy.logwarn("Error while set robot image for %s: %s", str(self.__current_master_label_name), str(e))

  def _on_robot_icon_changed(self, masteruri, path):
    '''
    One of the robot icons was chagned. Update the icon.
    '''
    master = self.getMaster(masteruri, False)
    if master:
      self._assigne_icon(master.mastername, resolve_url(path))

  def _callback_diagnostics(self, data):
    try:
      for diagnostic in data.status:
        if DIAGNOSTICS_AVAILABLE:
          self.diagnostics_signal.emit(diagnostic)
    except Exception as err:
      rospy.logwarn('Error while process diagnostic messages: %s'%err)
