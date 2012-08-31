import logging; logger = logging.getLogger("morse." + __name__)
import bge
import math
import mathutils
import morse.core.sensor

class ObjectDetectorClass(morse.core.sensor.MorseSensorClass):
    """ Object detector sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        self.add_property('_target', 'detector_target', 'Target')
        self.add_property('_threshold', 3.0, 'DetectionDistance')

        # Identify an object as the target of the detection
        try:
            target_name = self._target
            if target_name != '':
                scene = bge.logic.getCurrentScene()
                self._target_object = scene.objects[target_name]
                self._target_position_3d = morse.helpers.transformation.Transformation3d(self._target_object)
                logger.info("Using object '%s' as detection target", target_name)
        except KeyError as detail:
            self._target_object = None
            logger.error("Detection object not found!")



        self.local_data['x'] = 0.0
        self.local_data['y'] = 0.0
        self.local_data['z'] = 0.0
        self.local_data['orientation'] = mathutils.Quaternion()
        self.local_data['valid'] = False

        # reference for rotating world frame to sensor frame
        self.rot_w2s = self.blender_obj.worldOrientation

        logger.info("Component initialized, runs at %.2f Hz ", self.frequency)


    def default_action(self):
        """ Get the x, y, z, yaw, pitch and roll of the blender object that should be detected. """

        detected = False

        if self._target_object:
            # update the position of the target object
            self._target_position_3d.update(self._target_object)

            # for now we simply detect if we are closer than 4m
            if self.position_3d.distance(self._target_position_3d) < self._threshold:
                detected = True
                # get the transformation from sensor to object
                tf_sensor2object = self.position_3d.transformation3d_with(self._target_position_3d)

                # Store the data acquired by this sensor that could be sent
                #  via a middleware.
                self.local_data['x'] = tf_sensor2object.x
                self.local_data['y'] = tf_sensor2object.y
                self.local_data['z'] = tf_sensor2object.z
                self.local_data['orientation'] = tf_sensor2object.rotation

        self.local_data['valid'] = detected
