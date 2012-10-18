import logging; logger = logging.getLogger("morse." + __name__)

import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs');

import rospy
from geometry_msgs.msg import TwistStamped

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    component_name = component_instance.blender_obj.name

    # Add the new method to the component
    component_instance.output_functions.append(function)

    logger.setLevel(logging.INFO)

    # Generate one publisher and one topic for each component that is a sensor and uses post_message
    self._topics.append(rospy.Publisher(self.topic_name(component_instance), TwistStamped))

    # Extract the Middleware parameters
    # additional parameter should be a dict
    try:
        frame_id = mw_data[3].get("frame_id", "/world")
    except:
        frame_id = "/map"

    # create a new dictionary for this sensor if necessary
    if component_name not in self._properties:
        self._properties[component_name] = {}
    # store the frame ids in the dict
    self._properties[component_name]['frame_id'] = frame_id

    logger.info("Initialized the ROS twist sensor with frame_id '%s'",
                frame_id)



def post_twist(self, component_instance):
    """ Publish the data of the Velocity as a ROS TwistStamped message
    """
    component_name = component_instance.blender_obj.name
    frame_id = self._properties[component_name]['frame_id']
    try:
        publish = component_instance.local_data['valid']
    except:
        publish = True

    if publish:
        twistStamped = TwistStamped()
        twistStamped.header.stamp = rospy.Time.now()
        twistStamped.header.frame_id = frame_id
        twistStamped.twist.linear.x = component_instance.local_data['world_linear_velocity'][0]
        twistStamped.twist.linear.y = component_instance.local_data['world_linear_velocity'][1]
        twistStamped.twist.linear.z = component_instance.local_data['world_linear_velocity'][2]
        twistStamped.twist.angular.x = component_instance.local_data['angular_velocity'][0]
        twistStamped.twist.angular.y = component_instance.local_data['angular_velocity'][1]
        twistStamped.twist.angular.z = component_instance.local_data['angular_velocity'][2]

        for topic in self._topics:
            # publish the message on the correct topic    
            if str(topic.name) == self.topic_name(component_instance):
                topic.publish(twistStamped)
