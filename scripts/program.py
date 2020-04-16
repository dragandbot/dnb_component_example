#!/usr/bin/env python

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
	rospy.Service('add_two_numbers', AddTwoNumbers, handle)

	pub_component_status = rospy.Publisher('~status', ComponentStatus, queue_size=1, latch=True)
	
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		cm_status = ComponentStatus()
		cm_status.status_id = ComponentStatus().RUNNING
		pub_component_status.publish(cm_status)
		rate.sleep()