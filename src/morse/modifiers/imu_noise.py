import logging; logger = logging.getLogger("morse." + __name__)
import bge
import morse.modifiers.gaussian

from morse.core.modifier import MorseModifierClass

class MorseIMUNoiseClass(MorseModifierClass):

    gyro_std_dev = 0
    accel_std_dev = 0

    def register_component(self, component_name, component_instance, mod_data):
        """ Add the corresponding function to a component. """
        # Extract the information for this modifier
        # This will be tailored for each middleware according to its needs
        function_name = mod_data[1]

        try:
            # Get the reference to the function
            function = getattr(self, function_name)
        except AttributeError as detail:
            logger.error("%s. Check the 'component_config.py' file for typos" % detail)
            return

        if function_name == "noisify":
            component_instance.output_modifiers.append(function)
        else:
            logger.warning("Unknown function name for IMU Noise modifier. Check component_config.py file.")

        # Extract the Modifier parameters
        # First one is the gyroscope standard deviation
        # Second one is the accelerometer standard deviation
        try:
            self.gyro_std_dev = mod_data[2]
            self.accel_std_dev = mod_data[3]
        except:
            pass


    def noisify(self, component_instance):
        for i in range(0, 3):
            component_instance.local_data['angular_velocity'][i] = morse.modifiers.gaussian.gaussian(self.gyro_std_dev, component_instance.local_data['angular_velocity'][i])
            component_instance.local_data['linear_acceleration'][i] = morse.modifiers.gaussian.gaussian(self.accel_std_dev, component_instance.local_data['linear_acceleration'][i])

