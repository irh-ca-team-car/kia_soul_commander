#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

speed=0
brake=0
last_time= 0
pub = None
values = []
def callback(data):
    global last_time
    global speed
    global values
    values.append(data.data)
    if len(values) > 5:
        values=values[1::]
    change = max([-0.15,sum(values)/len(values)-0.3]) - brake
    print("change", change)
    seconds = rospy.get_time()
    delta_t=1#seconds-last_time
    last_time=seconds
    if speed <0:
        speed =0
    speed = change* delta_t + speed
    print("speed", speed)
    pub.publish(speed)

def callback_brake(data):
    global brake
    brake=data.data

def listener():
    global pub
    rospy.init_node('simulation', anonymous=True)
    pub = rospy.Publisher('car/speed/actual', Float64, queue_size=10)
    rospy.Subscriber("car/throttle", Float64, callback)
    rospy.Subscriber("car/brake",Float64,callback_brake)
    rospy.spin()

if __name__ == '__main__':
    listener()