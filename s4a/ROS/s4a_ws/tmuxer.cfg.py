
if not 'SIM' in globals():
	SIM = False
if not 'ARM' in globals():
	ARM = 's3a'
	
#TODO Better.
import platform
hostname = platform.node()
on_laptop = hostname = 'micuri-dell-laptop' and 'true' or 'false'

import subprocess
def run_for_stdout(str_cmd):
	r = subprocess.run(str_cmd.split(), stdout = subprocess.PIPE)
	return r.stdout.decode('utf-8')
on_rpi = run_for_stdout('lsb_release -i') == 'Distributor ID:	Raspbian\n'

arm_main = f'{ARM}_main'
if not on_rpi:
	if ARM == 's3a':
		arm_main = f'{ARM}_serial_main'

_settings = {
	'SIM': SIM,
	'ARM': ARM,
	'arm_main': arm_main,
	'on_laptop' : on_laptop,
}



_layout = [
	# Key is win name, value are panes.
	{
		'ros1':
		# Tuple is vert pane split, while list is horiz.
		(
			'main',
			['teleop', 'routine'],
		)
	},
	{
		'ros2':
		(
			['joint_echo', 'pose_echo'],
			['joy_echo', 'cmd'],
		)
	},
	{ 'joints': 'joints' },
	{
		'debug':
		(
			['debug1'],
		)
	},
	{ 'scoreboards': 'scoreboards' },
	{ 'playground': 'playground' },
]


# Key is target name, value is list of batches.
_targets = {
	# Done before everything.
	'__setup': [
		{
			**{
				f'debug{i}' : 'cd ../../FW/Servo_Motor_Ctrl/' for i in range(1, 2)
			},
		}
		
	],
	'build': [
		# Batch where key is pane where to execute,
		# value is multiline str of cmds.
		{
			'main': '''
				catkin_make
				''',
		},
	],
	'__pre_run': [
		{
			pane : 'source devel/setup.bash' for pane in [
				'main',
				'routine',
				'teleop',
				'joint_echo',
				'pose_echo',
				'joy_echo',
				'cmd',
				'joints',
				'scoreboards',
			]
		},
	],
	'geom': [
		{
			# Мultiline str: every line new cmd.
			# Backslash (\) before new line means continuation of line
			# NO_ENTER means that command is just pasted, not execute,
			# i.e. no ENTER will be send to tmux pane.
			'main': '''
				tmux_no_wait
				roslaunch s3a_main check_geom.launch
			''',
		},
	],	
	'run': [
		{
			# Мultiline str: every line new cmd.
			# Backslash (\) before new line means continuation of line
			# NO_ENTER means that command is just pasted, not execute,
			# i.e. no ENTER will be send to tmux pane.
			'main': '''
				tmux_no_wait
				roslaunch ${arm_main} main.launch \
					all_motors_sim:=${SIM} \
					small_screen:=${on_laptop}
				''',
			'teleop': '''
				sleep 10
				''',
			'playground': '''
				function list_controllers(){ \
					rosrun controller_manager controller_manager list; \
				}
				function traj(){ \
					rosrun common_teleop change_controller.py traj; \
				}
				#TODO
				#function unsim_j0(){ \
				#	rostopic pub -1 /b4a/sim_motors std_msgs/ByteMultiArray " \
				#	layout: \
				#	  dim: [] \
				#	  data_offset: 0 \
				#	data: [false, true, true, true, true, true] \
				#	" \
				#}
				tmux_exit NO_ENTER
				''',
		},
		{
			'routine': '''
				roslaunch common_teleop routines_teleop.launch name:=${ARM}
				''',
			'teleop': '''
				# Joypad must have analog on.
				#roslaunch arm_teleop jog_teleop.launch name:=${ARM}
				roslaunch arm_teleop servo_teleop.launch name:=${ARM}
				#roslaunch vision_teleop vision_teleop.launch name:=${ARM}
				''',
			'joint_echo': '''
				rostopic echo /joint_states
				''',
			'pose_echo': '''
				rostopic echo /tf_publish/beam00_base__to__beam3_hand_ee
				''',
			'joy_echo': '''
				rostopic echo /servo_server/delta_twist_cmds
				#rostopic echo /vision_teleop/delta_twist_cmds
				''',
			'cmd': '''
				rosrun moveit_commander moveit_commander_cmdline.py arm
				''',
			'joints': '''
				tail --follow \
					`roslaunch-logs`/robot_hardware_interface__joints.log
				''',
			'scoreboards': '''
				tail --follow \
					`roslaunch-logs`/motor_ctrl_serial_master__scoreboards.log
				''',
			
			'debug1': '''
				./waf debug_uart --port=1 NO_ENTER
			''',
		},
	]
}

# Do not wait in last batch.
last_batch = _targets['run'][-1]
no_wait_last_batch = {}
for pane, cmds in last_batch.items():
	cmds = 'tmux_no_wait\n' + cmds
	no_wait_last_batch[pane] = cmds
_targets['run'][-1] = no_wait_last_batch

_dependencies = {
	'build': ['__setup'],
	'geom': ['__setup', '__pre_run'],
	'run': ['__setup', '__pre_run'],
}

