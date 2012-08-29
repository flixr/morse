import logging; logger = logging.getLogger("morse." + __name__)
import bge
import morse.modifiers.gaussian
from math import radians, degrees
import mathutils

from morse.core.modifier import MorseModifierClass

class MorsePoseNoiseClass(MorseModifierClass):

    _pos_std_dev = 0.05
    _rot_std_dev = radians(5)

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
            logger.warning("Unknown function name for Pose Noise modifier. Check component_config.py file.")

        # Extract the Modifier parameters from the dictionary
        try:
            self._pos_std_dev = mod_data[2].get("pos_std", 0.05)
            self._rot_std_dev = mod_data[2].get("rot_std", radians(5))
        except:
            pass

        logger.info("Adding noise to Pose with standard deviations: position %.4f, rotation %.4f deg", \
                    self._pos_std_dev, degrees(self._rot_std_dev))


    def noisify(self, component_instance):
        # add noise on position
        for variable in ['x', 'y', 'z']:
            component_instance.local_data[variable] = \
                morse.modifiers.gaussian.gaussian(self._pos_std_dev, component_instance.local_data[variable])
        # add noise on roll, pitch, yaw angles
        # this is a very crude method, but sufficient for preliminary testing
        try:
            quat = component_instance.local_data['quat']
            euler = quat.to_euler()
            for i in range(0, 3):
                euler[i] = morse.modifiers.gaussian.gaussian(self._rot_std_dev, euler[i])
            component_instance.local_data['quat'] = euler.to_quaternion()
        except:
            for variable in ['roll', 'pitch', 'yaw']:
                component_instance.local_data[variable] = \
                    morse.modifiers.gaussian.gaussian(self._rot_std_dev, component_instance.local_data[variable])

