^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package multimaster_fkie
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.4 (2016-04-21)
------------------
* multimaster_fkie: added '/do_not_sync' parameter
  this allows to hide some topics/services, topic types, from
  synchronisation. It can be defined as string or as list.
* master_sync_fkie: fixed unnecessary update requests
  wrong timestamps leads to updates
* node_manager_fkie: added visualisation for not synchronized topics/services
* node_manager_fkie: add parameter to the order of publisher/subscriber in description dock
  new parameter: 'Transpose pub/sub description'
* node_manager_fkie: changed behaviour of description dock while update info
* node_manager_fkie: fixed deselection of text on context menu
* node_manager_fkie: fixed threading problem while searching for sync interfaces
* Contributors: Alexander Tiderko

0.5.3 (2016-04-01)
------------------
* node_manager_fkie: fix remote start
* Contributors: Alexander Tiderko

0.5.2 (2016-03-31)
------------------
* node_manager_fkie: fixed start process on remote hosts without Qt
* Contributors: Alexander Tiderko

0.5.1 (2016-03-23)
------------------
* Use ROS_HOSTNAME before ROS_IP.
  To determine which interface to use follow ROS master convention
  of ROS_HOSTNAME being used before ROS_IP.
  This is as per:
  http://wiki.ros.org/ROS/EnvironmentVariables#ROS_IP.2BAC8-ROS_HOSTNAME
* Contributors: Mike O'Driscoll, Timo Röhling

0.5.0 (2016-03-17)
------------------
New Features:
* node_manager_fkie: the start with different ROS_MASTER_URI
  sets now the ROS_HOSTNAME environment variable if a new masteruri was
  selected to start node_manager or master_discovery
* node_manager_fkie: added parameter to disable the highlighting of xml blocks
* node_manager_fkie: added ROS-Launch tags to context menu in XML editor
* node_manager_fkie: mark XML tag blocks
* node_manager_fkie: show the filename in the XML editor dialog title
* node_manager_fkie: close configuration items are now sorted
* node_manager_fkie: the confirmation dialog at exit can be deaktivated
  to stop all nodes and roscore or shutdown the host you can use the close
  button of each master
* node_manager_fkie: allow to shutdown localhost
* node_manager_fkie: shows 'advanced start' button also if the selected node laready runs
* default_cfg_fkie: added 'load_params_at_start' parameter.
  On start of default_cfg_fkie all parameters are loaded into ROS
  parameter server. If this parameter is set to `False` the parameter are
  loaded on first run of an included node.

Fixes:
* node_manager_fkie: fixed print XML content in echo_dialog
* node_manager_fkie: avoids the print of an error, while loads a wrongs file on start of the node_manager
* node_manager_fkie: fixed check of running remote roscore
* node_manager_fkie: fixed problem while echo topics on remote hosts
* node_manager_fkie: changed cursor position in XML editor after open node configuration
* node_manager_fkie: fixed replay of topics with array elements
* node_manager_fkie: fixed close process while start/stop nodes
* node_manager_fkie: fixed namespace of capability groups, fixed the missing leading SEP
* node_manager_fkie: fixed - avoid transmition of some included/changed but not needed files to remote host
* node_manager_fkie: fixed start node after a binary was selected from multiple binaries
* node_manager_fkie: removed "'now' FIX" while publish messages to topics
* node_manager_fkie: fixed log format on remote hosts
* master_discovery: fixed avg. network load calculation, added checks for some parameters
* multimaster_fkie: Set correct logging level to warning
* Contributors: Alexander Tiderko, Gary Servin

0.4.4 (2015-12-18)
------------------
* node_manager_fkie: fixed republish of array values in paraeter dialog
* node_manager_fkie: reviewed the name resolution
* node_manager_fkie: added an IP to hostname resolution
  it is usefull for detection of automatic master_sync start if an IP was
  entered while start of master_discovery
* node_manager_fkie: added a settings parameter 'start_sync_with_discovery'
  The start_sync_with_discovery determine the default behaviour to start
  master_sync with master_discover or not. This presets the 'Start sync'
  parameter in Start-dialog.
* node_manager_fkie: added an option to start master_sync with master_discovery
* node_manager_fkie: added network ID visualization
* node_manager_fkie: fixed joining from discovery dialog
* node_manager_fkie: fixed discovery dialog, which was broken after changes in master_discovery
* node_manager_fkie: highlighted the sync button in ROS network dock
* Contributors: Alexander Tiderko

0.4.3 (2015-11-30)
------------------
* node_manager_fkie: start rviz now as NO rqt plugin
* node_manager_fkie: fixed the sort of paramerter in `add parameter` dialog
* node_manager_fkie: adapt the chagnes in master_discovery_fkie
* node_manager_fkie: fixed the tooltip of the buttons in the description dock
* node_manager_fkie: stop /master_discovery node before poweroff host to avoid timout problems
* multimaster_fkie: reduced logs and warnings on stop nodes while closing node_manager
* node_manager_fkie: added a new button for call service
* node_manager_fkie: added a "copy log path to clipboard" button
* node_manager_fkie: fixed the displayed count of nodes with launch files in description dock
* node_manager_fkie: fixed errors showed while stop nodes on close
* multimaster_fkie: reduced logging of exceptions
* node_manager_fkie: added poweroff command to the host description
* node_manager_fkie: added tooltips to the buttons in description dock
* node_manager_fkie: replaced some icons
* node_manager_fkie: added advanced start link to set console format and loglevel while start of nodes
* node_manager_fkie: skip commented nodes while open a configuration for a selected node
* node_manager_fkie: fixed xml editor; some lines was hide
* node_manager_fkie: added ctrl+shift+slash to shortcuts for un/comment text in editor
   - some small changes in find dialog
* master_discovery_fkie: fixed compatibility to older versions
* master_fiscovery_fkie: integrated pull request `#24 <https://github.com/fkie/multimaster_fkie/issues/24>`_
  Thanks for creating the PR to @garyservin and @mikeodr!
  The change lets you define an interface by `~interface`, `ROS_IP` envar
  or append the interface to multicast group like
  226.0.0.0@192.168.101.10. The master_discovery then binds to the
  specified interface and creates also an unicast interface for active
  requests on communication problems or if `~robot_hosts` are defined.
  Now you can also disable the multicast communication by setting
  `~send_mcast` to false. In this case the requests are send to hosts
  defined in `~robot_hosts`.
* master_discovery_fkie: fixed the 'local' assignment while updateInfo()
* master_discovery_fkie: adopt some changes from pull request `#24 <https://github.com/fkie/multimaster_fkie/issues/24>`_
  Thanks to @garyservin for pull request `#24 <https://github.com/fkie/multimaster_fkie/issues/24>`_:
  * Don't exit if we're on localhost, just log a warning
  * Added support for different logging levels in master_monitor:
  currently all logs are marked as warnings, where some should be marked
  as errors.
* master_discovery_fkie: spaces and typos removed
* master_discovery_fkie: fixed link quality calculation
* Contributors: Alexander Tiderko

0.4.2 (2015-10-19)
------------------
* node_manager_fkie: added further files to change detection
* node_manager_fkie: fixed parameter dialog for some messages e.g. MarkerArray
* node_manager_fkie: shutdown now all nodes and roscore at exit (if selected)
* node_manager_fkie: changed diagnostic visualization
* node_manager_fkie: propagate the diagnostic color of a node to his group
* node_manager_fkie: update the description of selected node after a diagnostic message is recieved
* multimaster_fkie: added a possibility to set time on remote host
* node_manager_fkie: fixed the comparison of host time difference
* node_manager_fkie: added a warning if the time difference to remote host is greater than a defined value (default 3 sec)
* node_manager_fkie: added ControlModifier to package navigation
  Ctrl+DoubleClick:
  * History file: goto the package of the launch file
  * ..: goto root
  * folder: go only one step down, not until first config file
* node_manager_fkie: changed param template for parameter name in editor
* node_manager_fkie: added log button for remote master_discovery
  * show now only the screen log
* node_manager_fkie: fixed save/load in parameter dialog
* node_manager_fkie: fix load parameter with absolute path
* node_manager_fkie: added more info for error while set a parameter with None value
* node_manager_fkie: added icon for rqt plugin
* node_manager_fkie: fixed error which prevent display info and configuration dialogs
* node_manager_fkie: check now for changes of local binaries and ask for restart if these are changed
* node_manager_fkie: fixed problem while publishing to topic with lists and byte values
* node_manager_fkie: added support diagnostics_agg topic
* node_manager_fkie: added a remote script which does not use qt bindings
* master_discovery_fkie: fixed the updates of remote nodes registered on local master
* master_discovery_fkie: added @part to define interface with mcast group
* master_discovery_fkie: add posibility to specify the interface to use
* master_discovery_fkie: check for local ip addresses to avoid wrong warning messages
* Contributors: Alexander Tiderko

0.4.1 (2015-04-28)
------------------
* node_manager_fkie: fixed error while parsing list of lists in parameter dialog
* node_manager_fkie: added scrollarea for dynamic_reconfigure widget
* fixed the usage of new parameter in node_manager
* node_manager_fkie: fixed binary selection while 'add node'
* multimaster_fkie: fixed double log output
* node_manager_fkie: fix to enable the master list if a master_discavery was started
* node_manager_fkie: fixed recursive search
* multimaster_fkie: added network problem detection on remote hosts
* node_manager_fkie: older paramiko versions does not support get_pty parameter in exce_command
* node_manager_fkie: fixed stdout error while transfer files to remote host
* node_manager_fkie: ignore errors caused on after the echo dialog was closed
* node_manager_fkie: changed the color of illegal ros node names
* master_sync_fkie: Deprecate is_ignored_topic. Move new parameters to the end of the parameter list
* master_sync_fkie: Make configuration more granular
    allows filtering of specific subscribers or publishers
* Contributors: Alexander Tiderko

0.4.0 (2015-02-20)
------------------
* multimaster_fkie: discovery changed
  * reduced the amount of heartbeat messages for discovery
  * added fallback for environments with multicast problems
* node_manager_fkie: added log_level parameter to all nodes
* node_manager_fkie: fixed syntax highlightning
* node_manager_fkie: fix ssh handler
* node_manager_fkie: parameter changed in dialog "start master discovery"
* node_manager_fkie: fixes in parameter dialog
  * fixed filter in parameter dialog
  * fixed parser of the list values
  * update only changed values in ROS parameter server
* node_manager_fkie: default value for heartbeat changed to 0.5
* node_manager_fkie: improved the discovery dialog to detect masters using new methods
* node_manager_fkie: fixed the button view in the sync dialog
* node_manager_fkie: added a xml and yaml validation on save of a configuration files
* master_sync_fkie: fix the long wait time on first sync
* master_sync_fkie: fix annonce publisher about the AnyMsg subscribers
* master_discovery_fkie: discovery changed
  - reduced the amount of heartbeat messages for discovery
  - added fallback for environments with multicast problems
* master_discovery_fkie: added log_level parameter to all nodes
* master_discovery_fkie: changed discovery after the host was set to offline
* master_discovery_fkie: fixed a problem if more then one master discovery is running on the same host
* master_discovery_fkie: removed some python mistakes
* master_discovery_fkie: removed some debug output
* master_discovery_fkie: fixed change to offline state after a refresh service was called and host is not reachable
* master_discovery_fkie: fix set to offline state
* master_discovery_fkie: fixed link quality detection.
  The requests for each master are now stored, to detect the right count
  of messages that we have to receive.
* Contributors: Alexander Tiderko

0.3.18 (2015-02-18)
-------------------
* node_manager_fkie: fixed alt+space for context menu in xml editor
* node_maanger_fkie: removed sync+AnyMsg option, it is now sync with all messages
* node_manager_fkie: fix an error printed on close of echo dialog
* node_manager_fkie: fixed some ssh issues
* node_manager_fkie: enabled ssh compression
* node_manager_fkie: store user per host
* node_manager_fkie: added rviz to rqt menu
* node_manager_fkie: show now unknown topic types through the SSH connection
* node_manager_fkie: close running nodes on exit
* node_manager_fkie: fixed bug while creation of a new file in xml editor
* node_manager_fkie: added binary selection dialog to xml editor, if you add a node section using 'add tag' button
* node_manager_fkie: trap the errors printed to stderr in popen
* node_manager_fkie: fixed highlightning in editor
* master_sync: subscribers with None type are now subscribed as AnyType message
* Contributors: Alexander Tiderko

0.3.17 (2015-01-22)
-------------------
* node_manager_fkie: switch to local monitoring after connection problems to local master_discovery
* node_manager_fkie: added an update procedure to refresh discovered masters
  In same cases the messages, which are send on the shutdown of the
  master_discovery are not received by node_manager. To update the
  discovered list in node_manager the complete list of discoevered hosts
  will be requested, if the localhost master is added as new master.
* node_manager_fkie: fixed error while publishing to 'std_msgs/Empty'
* master_discovery_fkie: fixed discovery support for ipv6
* Contributors: Alexander Tiderko

0.3.16 (2014-12-08)
-------------------
* node_manager_fkie: fixed a problem with screen view
  The node_manager uses the /usr/bin/x-terminal-emulator to show the
  screen content of the nodes. To execute a command with arguments
  'konsole', 'xterm' uses -e, 'terminator', 'gnome-terminal' or
  'xfce4-terminal'use '-x'.
* Contributors: Alexander Tiderko

0.3.15 (2014-12-01)
-------------------
* node_manager_fkie: fixed sync button handling
* multimaster_fkie: removed some python mistakes
* node_manager_fkie: removed some python mistakes
* node_manager_fkie: fixed node selection in description dock
* node_manager_fkie: some icons changed
* node_manager_fkie: 'autoupdate' parameter added
  The autoupdate parameter disables the automatic requests. It is usefull
  for low bandwidth networks.
* node_manager_fkie: reduced remote parameter requests
* node_manager_fkie: added a republish functionality
  This function is accessible in extended info widget.
* node_manager_fkie: fix publish with rate slower one
  Updated the topic info. Added constants to message definition view.
* node_manager_fkie: restores the view of expanded capability groups after reload of a launch file
* node_managef_fkie: fix sidebar parameter selection
* node_manager_fkie: fixes in parameter dialog
  * fixed filter in parameter dialog
  * fixed parser of the list values
  * update only changed values in ROS parameter server
* Contributors: Alexander Tiderko

0.3.14 (2014-10-24)
-------------------
* node_manager_fkie: added a warning to capability table, if multiple configurations for the same node are loaded
* node_manager_fkie: remove now the configuration in capability table after a host was removed
* node_manager_fkie: fixed error while navigate in description panel
* node_manager_fkie: changed sidebar parameter handling (for start host dialog)
* node_manager_fkie: changed the handling on click the sync button in master list
* node_manager_fkie: fixed tooltip for recent loaded files
* node_manager_fkie: fixed problems in capability table with multi-launch-files for the same host and group
* CapabilityHeader: Keep indices of _data and controlWidget in sync when inserting new capabilities
* Fixed crash in master_list_model if IPv6 addresses are present on the host
* node_manager_fkie:manual link added
* node_manager_fkie: added args and remaps to change detection after reload a launch file
* node_manager_fkie: ignore namespace while display the Capabilities in Capabilities table
* node_manager_fkie: fixed some template tags in xml editor
* node_manager_fkie: stop nodes first while restart nodes after loading a launch file
* node_manager_fkie: added support of $(find ...) statement to add images in decription of capabilities
* node_manager_fkie: xmleditor - ask for save by pressing ESC
* node_manager_fkie: changed the update strategy for description dock
* node_manager_fkie: changed the update strategy for description dock
* node_manager_fkie: changed name creation for default configuration node
* node_manager_fkie: fixed blocked focus if a xmleditor was open
* node_manager_fkie: fixed highlighter problem in pyqt
* node_manager_fkie: improved respawn script
* node_manager_fkie: fixed handling of history files
* node_manager_fkie: mark line with problems in launch editor
* master_sync_fkie: reduced update notifications after registration of a subscriber
* Contributors: Alexander, Alexander Tiderko, Stefan Oßwald, Timo Röhling

0.3.13 (2014-07-29)
-------------------
* node_manager_fkie: fixed the button view in the sync dialog
* node_manager_fkie: added a xml and yaml validation on save of a configuration files
* node_manager_fkie: changed the navigation in info widget
* node_manager_fkie: raise launch dock after the settings are restored
* node_manager_fkie: show up directory while package selection
* node_manager_fkie: added comment/uncomment functionality
* node_manager_fkie: added caching for browsing in launch files
* node_manager_fkie: show also folder with additional config files
* node_manager_fkie: stores the xml editor geometry
* Contributors: Alexander Tiderko

0.3.12 (2014-07-08)
-------------------
* node_manager_fkie: fix instalation problem with missed .ui files
* node_manager_fkie: fixed ros master preparation
  Do not try to start ROS master on remote hosts for echo topics, if this
  host are not reachable.
* Contributors: Alexander Tiderko

0.3.11 (2014-06-04)
-------------------
* node_manager_fkie: replaced the rxconsole and rxgraph by rqt button to start rqt plugins related to selected master
* node_manager_fkie: added a setting docking window
* node_manager_fkie: hints on start problems fixed, if no screen is installed
* node_manager_fkie: added a dock widget and button which shows warning messages
* node_manager_fkie: select the topics and services of a node while tab change and not while node selection. This reduce the cpu load.
* node_manager_fkie: fixed detection of local host at start
* node_manager_fkie: fix the removing of local master at startup
* node_manager_fkie: added features to launch file view
  * Search for packages
  * rename files
  * copy files
* node_manager_fkie: do not wait in the discovery loop at shutdown
* node_manager_fkie: cancel buttons redesined, some titles renamed
* node_manager_fkie: reduced the displayed namespace of the topics and services in info area
* node_manager_fkie: added F4 and F3 shortcasts for aditing a configuration and show a screen of a node
* node_manager_fkie: fixed InteractionNeededError while starting nodes on remote hosts using run dialog.
* node_manager_fkie: added timestamps to each printed message
* node_manager_fkie: fix detailed message box. Close using ESC button.
* node_manager_fkie: reload root path in xml file view, if the current path was deleted
* node_manager_fkie: fixed include tag of dropped file in xml editor
* node_manager_fkie: added for each node respawn parameters
* node_manager_fkie: improve respawn script
  The new script correctly checks the exit code of the launched
  process and can limit the number of respawns for faulty
  nodes.
* node_manager_fkie: use -T for terminal emulator
  -T is compatible with more terminal emulators than -title
* node_manager_fkie: added handling for some of other configuration file types to launch file view
* Open terminal windows with x-terminal-emulator
  The /usr/bin/x-terminal-emulator symlink is available on Debian
  based systems and points to the default terminal emulator on
  the system. /usr/bin/xterm will be used as fallback.
* node_manager_fkie: changed side bar selection while start hosts
* node_manager_fkie: fixed the parameter handling of parameter with list type
* master_sync_fkie: fixed a block while connection problems
* master_discovery_fkie: added some error catches to solve problems with removing of interfaces
* master_discovery_fkie: fixed a short timestamp represantation
* default_cfg_fkie: added 'default_cfg/autostart/exclude' parameter to exclude nodes from autostart
* default_cfg_fkie: flush stdout before SIGKILL
  Otherwise, the error message may not reach the console output in time.
* default_cfg_fkie: added a console output for count of pending autostart nodes
* default_cfg_fkie: set autostart to False after all node are started
* default_cfg_fkie: added a reload service, to reload the configuration
* default_cfg_fkie: added for each node respawn parameters
* default_cfg_fkie: added a possibility for delayd start or after a published topic is available
* default_cfg_fkie: loads now without the private namespace of the default_cfg node
* default_cfg_fkie: removed BASH_ENV from environment while start with respawn script
* default_cfg_fkie: added an autostart option
* Contributors: Alexander, Alexander Tiderko, Sammy Pfeiffer, Timo Röhling

0.3.10 (2014-03-31)
-------------------
* node_manager_fkie: fixed the activation of the local monitoring. Fixed the cancelation in selection dialog.
* node_manager_fkie: added an indicator for running roslaunch server
* node_manager_fkie: fixed layout problems
* node_manager_fkie: dialog size of `start master_discovery` changed
* node_manager_fkie: added a side bar with checkitems in start host dialog
* node_manager_fkie: fixed remove entries in combonox of parameter dialog
* node_manager_fkie: remove comments in launch file fixed
* node_manager_fkie: added a check for changed files in parameter value
* node_manager_fkie: inform about changed files only on activating the main GUI
* node_manager_fkie: fixed search routine
* node_manager_fkie: fixed multiple entries in dialog for publishing to a new topic
* node_manager_fkie: added a context sensitive proposals in XML editor
* node_manager_fkie: enabled drag&drop action in xmleditor and launch view
* node_manager_fkie: added a button for quick insertion of launch tags
* node_manager_fkie: reduced the cpu load of echo dialog
* node_manager_fkie: added a line limit in echo dialog
* node_manager_fkie: fixed the processing of jobs after the `cancel` button was pressed
* node_manager_fkie: added a `reload global parameter` link
  - select the loaded row in launch dialog after loading the launch file
  with double click
* node_manager_fkie: fixed start nodes with ns
* node_maager_fkie: the launch files are now loaded in a thread, so they don't block
* node_manager_fkie: fixed duplicate detection of running and synchronized nodes
* node_manager_fkie: sync dialog extended by a new button to sync topics containing AnyMsg as type
* node_manager_fkie: cmd line output for registered parameter changed
* node_manager_fkie: removed project file
* node_manager_fkie: remember the used path in parameter dialog
* node_manager_fkie: changed the handling of localhost in machine tag of launchfile
* master_sync_fkie: fixed a bug which sometimes does not synchronized some topics
* master_discovery_fkie: unsubscribe from parameter at the end
* master_discovery_fkie: remove invalid roslaunch uris from ROS Master

0.3.9 (2013-12-12)
------------------
* node_manager_fkie: set node to warning state, if it not renning propertly because of problems with illegal name
* node_manager_fkie: fixed detailed_msg_box error
* node_manager_fkie: added highlighting for illegal ros names
* master_discovery_fkie: added warning outputs on errors
* multimaster_fkie: moved .gitignore to top level

0.3.8 (2013-12-10)
------------------
* node_manager_fkie: added support for /robot_icon parameter to show an image of the roboter
* node_manager_fkie: fixed handling of binary data in ROS parameter server
* node_manager_fkie: update robot image on cancel file selection dialog
* node_manager_fkie: can now change the robot image by double-click on robot image
* node_manager_fkie: added autoselect corresponding topics and services on node selection
* node_manager_fkie: reduced timestamp updates, if node_manager is not active
* multimaster_fkie: added a possibility to deaktivate the multicast heart bearts
* node_manager_fkie: selection dialog extended by an description label
* node_manager_fkie: handling of included files chagned, to avoid errors if a package was not found
* node_manager_fkie: buttons of the discovery widged chagned
* node_manager_fkie: control buttons redesigned
* node_manager_fkie: added 'Do not display this warning again' button to warning message
* node_manager_fkie: fixed deleting of not reachable hosts
* node_manager_fkie: fixed wrong reference in sync_dialog
* node_manager_fkie: fixed copy mode (Ctrl+C copy now first column, Ctrl+X: type or value)
* node_manager_fkie: update launch file view after loading launch file
* node_manager_fkie: fixed echo dialog (icons, additional info)
* node_manager_fkie: added ROS_NAMESPACE environment parameter to launch process to handle some cases, e.g. rqt_cpp plugins
* node_manager_fkie: fixed watching for changes in included files
* node_manager_fkie: Delete key deletes now the selected history launch file
* node_manager_fkie: reduced window size
* node_manager_fkie: ignore empty 'capability_group' values
* multimaster_fkie: catkin_lint inspired fixes, thanks @roehling
* node_manager_fkie: fixed help call in the console
* node_manager_fkie: fix detection for included files
* node_manager_fkie: fixed open sync dialog from info panel
* node_manager_fkie: added a yaml highlighter
* node_manager_fkie: argparse integrated
* node_manager_fkie: fixed lower compare of topic and service names
* node_manager_fkie: fix - use now sensetive comparison of node names
* node_manager_fkie: fixed launch file browsing
* node_manager_fkie: fixed skipped display messages on latched topics
* master_sync_fkie: added sync for subscriber with AnyMsg, e.g relay (topic_tools), if local a publisher with known type is available
* master_discovery_fkie: added output info about approx. netload
* master_discovery_fkie: description in der package.xml changed
* default_cfg_fkie: fixed forward error to service caller
* default_cfg_fkie: ignore empty 'capability_group' values

0.3.7 (2013-10-17)
------------------
* node_manager_fkie: fixed start button description
* node_manager_fkie: added an info button
* node_manager_fkie: changed calling of sync dialog
* node_manager_fkie: showing duplicate nodes fixed
* multimaster_fkie: fixed problems with resolving service types while sync
  while synchronization not all topics and services can be synchronized
  because of filter or errors. A detection for this case was added.
* node_manager_fkie: added user selection for remote hosts
* node_manager_fkie: fixed some paths
* node_manager_fkie: added SAVE and LOAD buttons to parameter dialog
* node_manager_fkie: fixed start nodes in multimaster on the same host
* node_manager_fkie: replaced the sync checkbox in masterlist by a sync icon
* node_manager_fkie: fixed filtering topics, services and parameter
* node_manager_fkie: buttons resized
* node_manager_fkie: added missed start parameter to master_sync
* node_manager_fkie: removed some unneeded borders in gui
* node_manager_fkie: fix loading launch file
* node_manager_fkie: fixed parameter groups
* node_manager_fkie: added new interface of dynamic_reconfigure
* node_manager_fkie: show node_manager window maximized, if the screen is small
* node_manager_fkie: fixed raise conditions
* node_manager_fkie: added filter to selected dialog and changed selection behavior
* node_manager_fkie: fix node matching
* node_manager_fkie: fixed absolute path in env of the launch file
* master_discovery_fkie: do not publish not resolvable ROS MASTER URI to own ROS network
* default_cfg_fkie: fixed parameter groups

0.3.6 (2013-09-17)
------------------
* node_manager_fkie: added a notifiaction, if `use_sim_time` parameter is set to true
* node_manager_fkie: added some control elements to node/host description
* node_manager_fkie: fix load launch file
* node_manager_fkie: fix filter in paramter dialog
* node_manager_fkie: fixed do not store the launch file on error
* node_manager_fkie: the minimum size of the parameter dialog increased
* node_manager_fkie: update the capability group of the node using the ROS parameter server, if no launch file is loaded
* node_manager_fkie: fixed cancel loading of the launch file, on cancel input args
  node_manager_fkie: do not restart anonymous nodes on relaod launch file
  node_manager_fkie: fixed closing of the remote default configs on same host but other roscore
* node_manager_fkie: resize the node_manager window on small
* node_manager_fkie: changed the intepretation of the group description
* node_manager_fkie: remove not existing remote node information. In case of restarting a ROS node without stopn a running node.
* node_manager_fkie: fixed buttons description
* node_manager_fkie: fixed change detection in included files
* node_manager_fkie: add detection of changes in the reloaded launch file and restart affected nodes
* node_manager_fkie: fixed clear_params
* multimaster_msgs_fkie: added SyncServiceInfo message to detect changes on services
* master_sync_fkie: kill the own ros node on error while load interface to inform the user in node_manager about errors
* master_discovery_fkie: fixed the origin master uri for services
* master_discovery_fkie: fixed the result of the synchronized nodes (handle the restart of the nodes without stop the running node)
* master_discovery_fkie: fixed the test of local changes -> not all changes are propageted to other hosts
* default_cfg_fkie: changed the intepretation of the group description

0.3.5 (2013-09-06)
------------------
* node_manager_fkie: fixed launch selection for favirites with same launch file name
* node_manager_fkie: fixed process id view of nodes for multiple sync hosts
* master_sync_fkie: fixed a brocken connection after desync

0.3.4 (2013-09-05)
------------------
* node_manager_fkie: fixed file paths (removed warnings in file_watcher)
* node_manager_fkie: clear cached package names on refreshing launch file view
* node_manager_fkie: capability_group parameter can now be defined in a namespace
* node_manager_fkie: fixed pakage_name result
  added caching for package_name results
* default_cfg_fkie: capability_group parameter can now be defined in a namespace

0.3.3 (2013-09-04)
------------------
* node_manager_fkie: Parse package.xml for name
  Although package folders should have the same name as the
  package, some packages (e.g. swig-wx) violate this.
  Thus, we use catkin_pkg.package.parse_package to parse
  the package.xml and look for the <name> tag, which
  contains the correct package name.
* node_manager_fkie: Install data files without executable bit
* node_manager_fkie: added a button to hide the dock widgets
* node_manager_fkie: added a question dialog to start the synchronization with a loaded config, if any exists
* node_manager_fkie: increased timeout for transfer of parameter while start of nodes
* node_manager_fkie: fixed node name creation for publishing of topics
* node_manager_fkie: fixed start of master_sync with interface file
* node_manager_fkie: removed some exeption for pyqt workaround
* node_manager_fkie: added a warning in paramter dialog
* node_manager_fkie: fixed names, preselect all files to reload after a file was changed
* node_manager_fkie: added a buttons to save and load configurations
* node_manager_fkie: show the parent of the src-folder
* node_manager_fkie: plugin renamed
* node_manager_fkie: fixed finish function to stop the running timer
* node_manager_fkie: file watcher updated, changes now notified once for all master
* multimaster_fkie: .gitignore changed
* node_manager_fkie: don't ask for argv's while reloading
* node_manager_fkie: fixed a problem while launching a default cfg nodes
* node_manager_fkie: searching for packages in rundialog after dialog opened
* node_manager_fkie: fixed waiting for roscore
* node_manager_fkie: added the default group for system nodes, fixed an often update problem
* node_manager_fkie: fixed problem while openning an editor
* node_manager_fkie: increased the wait for ROS Master
* node_manager_fkie: added the possibility to enter a varible count of list entries while calling a service or publishing to a topic
* node_manager_fkie: changed the handling while close multiple configurations
* node_manager_fkie: added the parameter as pkg:// URL to launch a default_cfg at start of node_manager
* multimaster_fkie: (*) added additional filtered interface to master_discovery rpc-server to get a filtered MasterInfo and reduce the load on network.
  (*) added the possibility to sync remote nodes using ~sync_remote_nodes parameter
* node_manager_fkie: added a possibility to create a new files
* node_manager_fkie: fixed error while browsing in launch files
* node_manager_fkie: (1) added a button to transfer launch files to remote machines,
  (2) upgraded the editor for sync dialog
  (3) added more info to progress bars
* node_manager_fkie: limited displaying frequency for echo dialog
* node_manager_fkie: limited the displayed messages in echo widget
* node_manager_fkie: fixed a problem while launching a default cfg nodes
* master_sync_fkie: added support to ignore nodes/topic/services of selected hosts
* master_sync_fkie: fixed ignore hosts, some topics sync ignores
* master_discovery_fkie: fixed out, if the ROS_MASTER_URI refs to 'localhost'
* master_discovery_fkie: fixed the load interface
