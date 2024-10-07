import rospy
from std_msgs.msg import UInt16MultiArray

def pwm_publisher():
    # Create a Publisher object
    pub = rospy.Publisher('pwm_values', UInt16MultiArray, queue_size=10)
    rospy.init_node('output_integrator', anonymous=True)
    rate = rospy.Rate(1000)  # 1 kHz

    while not rospy.is_shutdown():
        # Create a message and populate it
        msg = UInt16MultiArray()
        msg.data = [1.1, 2.2, 3.3, 4.4, 5.5]  # Example data with high precision

        # Publish the message
        pub.publish(msg)
        rospy.loginfo("Published: %s", msg.data)
        rate.sleep()

def mpc_callback(msg):
    # Callback function to handle incoming messages
    rospy.loginfo("Received array: %s", msg.data)

def rl_callback(msg):
    # Callback function to handle incoming messages
    rospy.loginfo("Received array: %s", msg.data)

def mpc_subscriber():
    # Create a Subscriber object
    rospy.Subscriber('mpc_control_values', UInt16MultiArray, mpc_callback)

def rl_subscriber():
    # Create a Subscriber object
    rospy.Subscriber('rl_control_values', UInt16MultiArray, rl_callback)

def main():
    rospy.init_node("output_integrator", anonymous=True)
    
    pwm_publisher_thread = rospy.Thread(target=pwm_publisher)
    mpc_subscriber_thread = rospy.Thread(target=mpc_subscriber)
    rl_subscriber_thread = rospy.Thread(target=rl_subscriber)
    
    pwm_publisher_thread.start()
    mpc_subscriber_thread.start()
    rl_subscriber_thread.start()
    
    rospy.spin()

if __name__ == '__main__':
    main()