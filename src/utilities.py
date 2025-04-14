from scipy.spatial.transform import Rotation


def euler_to_quat(x, y, z):
        """
        Parameters:
            x (float): Rotation angle around the x-axis in radians.
            y (float): Rotation angle around the y-axis in radians.
            z (float): Rotation angle around the z-axis in radians.
        Returns:
            numpy.ndarray: A quaternion represented as [w, x, y, z].
        """
        
        rot = Rotation.from_euler('xyz', [x, y, z], degrees=False)
        rot_quat = rot.as_quat(scalar_first=True)  # [w, x, y, z]
        
        
        return rot_quat