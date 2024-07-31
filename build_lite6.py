#!/usr/bin/env python
# Copy this file to ~/.local/lib/python3.8/site-packages/rtbdata/xacro
import numpy as np
from roboticstoolbox.robot.Robot import Robot
from spatialmath import SE3


class Lite6(Robot):
    """
    Class that imports a Panda URDF model

    ``Panda()`` is a class which imports a Franka-Emika Panda robot definition
    from a URDF file.  The model describes its kinematic and graphical
    characteristics.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.URDF.Panda()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration
    - qs, arm is stretched out in the x-direction
    - qn, arm is at a nominal non-singular configuration

    .. codeauthor:: Jesse Haviland
    .. sectionauthor:: Peter Corke
    """

    def __init__(self):

        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "lite6.urdf"
        )

        super().__init__(
            links,
            name=name,
            manufacturer="UFactory",
            # gripper_links=links[9],
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        # self.grippers[0].tool = SE3(0, 0, 0.1034)

        self.qdlim = np.array(
            [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100]
        )

        self.qr = np.array([0.0, 0.2792, 0.8499, 0.0, 0.5724, 0.0])
        self.qz = np.zeros(7)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover

    r = Lite6()

    # r.qr

    qz = np.zeros(6)
    qr = np.array([0.0, 0.2792, 0.8499, 0.0, 0.5724, 0.0])
    print(r.fkine(qr))

    # for link in r.grippers[0].links:
    #     print(link)
