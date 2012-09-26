import logging; logger = logging.getLogger("morse." + __name__)
import math
import morse.core.sensor

class AltitudeClass(morse.core.sensor.MorseSensorClass):
    """ Altitude sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)
        
        self.add_property('_offset', 0.0, 'AltitudeOffset')

        self.local_data['height'] = self._offset

        logger.info('Component initialized')


    def default_action(self):
        """ Get the altitude of the blender object. """

        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['height'] = -(self.position_3d.z - self._offset)
