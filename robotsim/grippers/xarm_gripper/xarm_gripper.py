import os
import numpy as np
import modeling.geometricmodel as gm
import robotsim._kinematics.jlchain as jl


class XArmGripper(object):

    def __init__(self, pos=np.zeros(3), rotmat=np.eye(3)):
        this_dir, this_filename = os.path.split(__file__)
        self.pos = pos
        self.rotmat = rotmat
        # joints
        # - lft_outer
        self.lft_outer = jl.JLChain(pos=self.pos, rotmat=self.rotmat, homeconf=np.zeros(2), name='lft_outer')
        self.lft_outer.jnts[1]['loc_pos'] = np.array([0, .035, .059098])
        self.lft_outer.jnts[1]['rngmin'] = .0
        self.lft_outer.jnts[1]['rngmax'] = .85  # TODO change min-max to a tuple
        self.lft_outer.jnts[1]['loc_motionax'] = np.array([1, 0, 0])
        self.lft_outer.jnts[2]['loc_pos'] = np.array([0, .035465, .042039])  # passive
        self.lft_outer.jnts[2]['rngmin'] = .0
        self.lft_outer.jnts[2]['rngmax'] = .85  # TODO change min-max to a tuple
        self.lft_outer.jnts[2]['loc_motionax'] = np.array([-1, 0, 0])
        # - lft_inner
        self.lft_inner = jl.JLChain(pos=self.pos, rotmat=self.rotmat, homeconf=np.zeros(1), name='lft_inner')
        self.lft_inner.jnts[1]['loc_pos'] = np.array([0, .02, .074098])
        self.lft_inner.jnts[1]['rngmin'] = .0
        self.lft_inner.jnts[1]['rngmax'] = .85  # TODO change min-max to a tuple
        self.lft_inner.jnts[1]['loc_motionax'] = np.array([1, 0, 0])
        # - rgt_outer
        self.rgt_outer = jl.JLChain(pos=self.pos, rotmat=self.rotmat, homeconf=np.zeros(2), name='rgt_outer')
        self.rgt_outer.jnts[1]['loc_pos'] = np.array([0, -.035, .059098])
        self.rgt_outer.jnts[1]['rngmin'] = .0
        self.rgt_outer.jnts[1]['rngmax'] = .85  # TODO change min-max to a tuple
        self.rgt_outer.jnts[1]['loc_motionax'] = np.array([-1, 0, 0])
        self.rgt_outer.jnts[2]['loc_pos'] = np.array([0, -.035465, .042039])  # passive
        self.rgt_outer.jnts[2]['rngmin'] = .0
        self.rgt_outer.jnts[2]['rngmax'] = .85  # TODO change min-max to a tuple
        self.rgt_outer.jnts[2]['loc_motionax'] = np.array([1, 0, 0])
        # - rgt_inner
        self.rgt_inner = jl.JLChain(pos=self.pos, rotmat=self.rotmat, homeconf=np.zeros(1), name='rgt_inner')
        self.rgt_inner.jnts[1]['loc_pos'] = np.array([0, -.02, .074098])
        self.rgt_inner.jnts[1]['rngmin'] = .0
        self.rgt_inner.jnts[1]['rngmax'] = .85  # TODO change min-max to a tuple
        self.rgt_inner.jnts[1]['loc_motionax'] = np.array([-1, 0, 0])
        # links
        # - lft_outer
        self.lft_outer.lnks[0]['name'] = "xarm_gripper_base"
        self.lft_outer.lnks[0]['loc_pos'] = np.zeros(3)
        self.lft_outer.lnks[0]['com'] = np.array([-0.00065489, -0.0018497, 0.048028])
        self.lft_outer.lnks[0]['mass'] = 0.5415
        self.lft_outer.lnks[0]['meshfile'] = os.path.join(this_dir, "meshes", "base_link.stl")
        self.lft_outer.lnks[1]['name'] = "left_outer_knuckle"
        self.lft_outer.lnks[1]['loc_pos'] = np.zeros(3)
        self.lft_outer.lnks[1]['com'] = np.array([2.9948e-14, 0.021559, 0.015181])
        self.lft_outer.lnks[1]['mass'] = 0.033618
        self.lft_outer.lnks[1]['meshfile'] = os.path.join(this_dir, "meshes", "left_outer_knuckle.stl")
        self.lft_outer.lnks[1]['rgba'] = [.2, .2, .2, 1]
        self.lft_outer.lnks[2]['name'] = "left_finger"
        self.lft_outer.lnks[2]['loc_pos'] = np.zeros(3)
        self.lft_outer.lnks[2]['com'] = np.array([-2.4536e-14, -0.016413, 0.029258])
        self.lft_outer.lnks[2]['mass'] = 0.048304
        self.lft_outer.lnks[2]['meshfile'] = os.path.join(this_dir, "meshes", "left_finger.stl")
        self.lft_outer.lnks[2]['rgba'] = [.2, .2, .2, 1]
        # - lft_inner
        self.lft_inner.lnks[1]['name'] = "left_inner_knuckle"
        self.lft_inner.lnks[1]['loc_pos'] = np.zeros(3)
        self.lft_inner.lnks[1]['com'] = np.array([2.9948e-14, 0.021559, 0.015181])
        self.lft_inner.lnks[1]['mass'] = 0.033618
        self.lft_inner.lnks[1]['meshfile'] = os.path.join(this_dir, "meshes", "left_inner_knuckle.stl")
        self.lft_inner.lnks[1]['rgba'] = [.2, .2, .2, 1]
        # - rgt_outer
        self.rgt_outer.lnks[1]['name'] = "right_outer_knuckle"
        self.rgt_outer.lnks[1]['loc_pos'] = np.zeros(3)
        self.rgt_outer.lnks[1]['com'] = np.array([-3.1669e-14, -0.021559, 0.015181])
        self.rgt_outer.lnks[1]['mass'] = 0.033618
        self.rgt_outer.lnks[1]['meshfile'] = os.path.join(this_dir, "meshes", "right_outer_knuckle.stl")
        self.rgt_outer.lnks[1]['rgba'] = [.2, .2, .2, 1]
        self.rgt_outer.lnks[2]['name'] = "right_finger"
        self.rgt_outer.lnks[2]['loc_pos'] = np.zeros(3)
        self.rgt_outer.lnks[2]['com'] = np.array([2.5618e-14, 0.016413, 0.029258])
        self.rgt_outer.lnks[2]['mass'] = 0.048304
        self.rgt_outer.lnks[2]['meshfile'] = os.path.join(this_dir, "meshes", "right_finger.stl")
        self.rgt_outer.lnks[2]['rgba'] = [.2, .2, .2, 1]
        # - rgt_inner
        self.rgt_inner.lnks[1]['name'] = "right_inner_knuckle"
        self.rgt_inner.lnks[1]['loc_pos'] = np.zeros(3)
        self.rgt_inner.lnks[1]['com'] = np.array([1.866e-06, -0.022047, 0.026133])
        self.rgt_inner.lnks[1]['mass'] = 0.023013
        self.rgt_inner.lnks[1]['meshfile'] = os.path.join(this_dir, "meshes", "right_inner_knuckle.stl")
        self.rgt_inner.lnks[1]['rgba'] = [.2, .2, .2, 1]
        # reinitialize
        self.lft_outer.reinitialize()
        self.lft_inner.reinitialize()
        self.rgt_outer.reinitialize()
        self.rgt_inner.reinitialize()

    def fix_to(self, pos, rotmat, angle=None):
        self.pos = pos
        self.rotmat = rotmat
        if angle is not None:
            self.lft_outer.jnts[1]['motion_val'] = angle
            self.lft_outer.jnts[2]['motion_val'] = self.lft_outer.jnts[1]['motion_val']
            self.lft_inner.jnts[1]['motion_val'] = self.lft_outer.jnts[1]['motion_val']
            self.rgt_outer.jnts[1]['motion_val'] = self.lft_outer.jnts[1]['motion_val']
            self.rgt_outer.jnts[2]['motion_val'] = self.lft_outer.jnts[1]['motion_val']
            self.rgt_inner.jnts[1]['motion_val'] = self.lft_outer.jnts[1]['motion_val']
        self.lft_outer.fix_to(self.pos, self.rotmat)
        self.lft_inner.fix_to(self.pos, self.rotmat)
        self.rgt_outer.fix_to(self.pos, self.rotmat)
        self.rgt_inner.fix_to(self.pos, self.rotmat)

    def gen_stickmodel(self, name='xarm_gripper_stickmodel'):
        stickmodel = gm.StaticGeometricModel(name=name)
        self.lft_outer.gen_stickmodel().attach_to(stickmodel)
        self.lft_inner.gen_stickmodel().attach_to(stickmodel)
        self.rgt_outer.gen_stickmodel().attach_to(stickmodel)
        self.rgt_inner.gen_stickmodel().attach_to(stickmodel)
        return stickmodel

    def gen_meshmodel(self, name='xarm_gripper_meshmodel'):
        meshmodel = gm.StaticGeometricModel(name=name)
        self.lft_outer.gen_meshmodel().attach_to(meshmodel)
        self.lft_inner.gen_meshmodel().attach_to(meshmodel)
        self.rgt_outer.gen_meshmodel().attach_to(meshmodel)
        self.rgt_inner.gen_meshmodel().attach_to(meshmodel)
        return meshmodel

    def fk(self, angle):
        """
        lft_outer is the only active joint, all others mimic this one
        :param: angle, radian
        """
        if self.lft_outer.jnts[1]['rngmin'] <= angle <= self.lft_outer.jnts[1]['rngmax']:
            self.lft_outer.jnts[1]['motion_val'] = angle
            self.lft_outer.jnts[2]['motion_val'] = self.lft_outer.jnts[1]['motion_val']
            self.lft_inner.jnts[1]['motion_val'] = self.lft_outer.jnts[1]['motion_val']
            self.rgt_outer.jnts[1]['motion_val'] = self.lft_outer.jnts[1]['motion_val']
            self.rgt_outer.jnts[2]['motion_val'] = self.lft_outer.jnts[1]['motion_val']
            self.rgt_inner.jnts[1]['motion_val'] = self.lft_outer.jnts[1]['motion_val']
            self.lft_outer.fk()
            self.lft_inner.fk()
            self.rgt_outer.fk()
            self.rgt_inner.fk()
        else:
            raise ValueError("The angle parameter is out of range!")


if __name__ == '__main__':
    import visualization.panda.world as wd
    import modeling.geometricmodel as gm

    base = wd.World(campos=[2, 0, 1], lookatpos=[0, 0, 0])
    gm.gen_frame().attach_to(base)
    # for angle in np.linspace(0, .85, 8):
    #     xag = XArmGripper()
    #     xag.fk(angle)
    #     xag.gen_meshmodel().attach_to(base)
    xag = XArmGripper()
    xag.fk(.85)
    xag.gen_meshmodel().attach_to(base)
    base.run()
