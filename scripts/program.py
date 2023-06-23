#!/usr/bin/env python3

import rospy

from dnb_component_example.srv import *
from dnb_msgs.msg import *

def handle(request):

	# get param gets in this case a string value
	use_offset = str(rospy.get_param('~use_offset', "true" )).lower() == "true"

	if use_offset:
		offset = int(rospy.get_param('~offset', "0" )) # Read the ROS parameter coming from the launch file in the same workspace as the node
	else:
		offset = 0

	response = AddTwoNumbersResponse()
	response.result = request.number1 + request.number2 + offset
	return response

if __name__ == "__main__":
	rospy.init_node('dnb_component_example')

	# INFO: Creates a service with the name 'add_two_numbers' and the interface AddTwoNumbers from the .srv
	#       file defined in the srv/ folder. Service is blocking and can be executed within the ROS system.
	#       A good practice is to create services or topics which can be later accessed from
	#       drag&bot Function Blocks.
	rospy.Service('add_two_numbers', AddTwoNumbers, handle)

	# INFO: Will publish the component state frequently and will be read by the drag&bot Component-Manager.
	#       The ComponentStatus() message consists of an status_id and a status_msg. When the component is
	#       in a RUNNING state drag&bot is fully usable. Whenever the Component will reach an ERROR state
	#       drag&bot will stop all executed programs and the status_msg will be shown to the user in
	#       the Component-Manager view of drag&bot.
	#
	#       The full msg interface can be found here:
	#       https://github.com/dragandbot/dragandbot_common/blob/devel/dnb_msgs/msg/ComponentStatus.msg
	pub_component_status = rospy.Publisher('~status', ComponentStatus, queue_size=1, latch=True)
	
	# INFO: The rospy.Rate() set the speed of the following while loop in Hz. Set this value as really necessary.
	#       Setting this value too high with very performance heavy tasks will influence system performance significantly.
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		cm_status = ComponentStatus()

		# ComponentStatus - RUNNING
		cm_status.status_id = ComponentStatus().RUNNING
		pub_component_status.publish(cm_status)

		# ComponentStatus - ERROR: Execute whenever a condition is reached where the component health status is not given
		#               or e.g. a connection to a hardware component is list.
		# cm_status.status_id = ComponentStatus().ERROR
		# pub_component_status.publish(cm_status)
		rate.sleep()