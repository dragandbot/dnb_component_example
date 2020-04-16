#!/usr/bin/env python

import rospy

from dnb_component_example.srv import *
from dnb_msgs.msg import *

def handle(request):
	response = AddTwoNumbersResponse()
	response.result = request.number1 + request.number2
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