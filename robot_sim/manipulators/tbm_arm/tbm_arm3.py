import os
import math
import numpy as np
import basis.robot_math as rm
import robot_sim._kinematics.jlchain as jl
import robot_sim.manipulators.manipulator_interface as mi


class TBMArm(mi.ManipulatorInterface):

    def __init__(self, pos=np.zeros(3), rotmat=np.eye(3), homeconf=np.zeros(7), name='tbm_arm', enable_cc=True):
        super().__init__(pos=pos, rotmat=rotmat, name=name)
        this_dir, this_filename = os.path.split(__file__)
        self.jlc = jl.JLChain(pos=pos, rotmat=rotmat, homeconf=homeconf, name=name)

        link_width = np.zeros(8)
        link_width[0] = 0.5
        link_width[1] = 0.51
        link_width[2] = 0.43
        link_width[3] = 0.41
        link_width[4] = 0.375
        link_width[5] = 0.26
        link_width[6] = 0.21
        link_width[7] = 0.275
        link2_length = 0.95
        link4_length = 1.1
        link6_length = 0.48
        slider_base_length = 4
        slider_base_thickness = 0.05
        slider_base_offset = 0.35

        self.jlc.jnts[1]['loc_pos'] = np.array([0, 0, 0])
        self.jlc.jnts[1]['loc_rotmat'] = rm.rotmat_from_euler(0, 0, 0)
        self.jlc.jnts[1]['type'] = 'prismatic'
        self.jlc.jnts[1]['loc_motionax'] = np.array([1, 0, 0])

        self.jlc.jnts[2]['loc_pos'] = np.array([0, 0, 0])  # jnts1坐标与基坐标的相对位置
        self.jlc.jnts[2]['loc_motionax'] = np.array([0, 0, 1])
        self.jlc.jnts[2]['motion_rng'] = [-math.radians(30), math.radians(30)]

        self.jlc.jnts[3]['loc_pos'] = np.array([0, 0, 0])  # jnts2坐标与jnts1坐标的相对位置
        self.jlc.jnts[3]['loc_motionax'] = np.array([1, 0, 0])
        # self.jlc.jnts[3]['motion_rng'] = [-2*math.pi, 2*math.pi]

        self.jlc.jnts[4]['loc_pos'] = np.array([link2_length, 0, 0])
        self.jlc.jnts[4]['loc_motionax'] = np.array([0, 0, 1])
        self.jlc.jnts[4]['motion_rng'] = [-math.radians(90), math.radians(90)]

        self.jlc.jnts[5]['loc_pos'] = np.array([0, 0, 0])
        self.jlc.jnts[5]['loc_motionax'] = np.array([1, 0, 0])
        # self.jlc.jnts[5]['motion_rng'] = [-2*math.pi, 2*math.pi]

        self.jlc.jnts[6]['loc_pos'] = np.array([link4_length, 0, 0])
        self.jlc.jnts[6]['loc_motionax'] = np.array([0, 0, 1])
        self.jlc.jnts[6]['motion_rng'] = [-math.radians(115), math.radians(115)]

        self.jlc.jnts[7]['loc_pos'] = np.array([0, 0, 0])
        self.jlc.jnts[7]['loc_motionax'] = np.array([1, 0, 0])

        # links
        self.jlc.lnks[0]['name'] = "sliderbase"
        self.jlc.lnks[0]['loc_pos'] = np.array(
            [slider_base_length / 2 - link_width[1] / 2, 0, -slider_base_offset])
        self.jlc.lnks[0]['meshfile'] = gm.gen_box(extent=[slider_base_length, link_width[0], slider_base_thickness])
        self.jlc.lnks[0]['rgba'] = [.35, .35, .35, 0.5]
        # self.jlc.lnks[0]['mass'] = 1.4
        # self.jlc.lnks[0]['com'] = np.array([-.02131, .000002, .044011])

        self.jlc.lnks[1]['name'] = "j1"
        self.jlc.lnks[1]['loc_pos'] = np.array([0, 0, link_width[1] / 2])
        self.jlc.lnks[1]['loc_rotmat'] = rm.rotmat_from_euler(0, math.pi / 2, 0)
        self.jlc.lnks[1]['meshfile'] = gm.gen_stick(spos=np.array([0, 0, 0]),
                                                    epos=np.array([link_width[1], 0, 0]),
                                                    thickness=link_width[1])
        self.jlc.lnks[1]['rgba'] = [.5, .5, .5, 0.5]

        self.jlc.lnks[2]['name'] = "j2"
        self.jlc.lnks[2]['loc_pos'] = np.array([link_width[1] / 2, 0, 0])
        cylinder_height = link2_length - link_width[1] / 2 - link_width[3] / 2
        self.jlc.lnks[2]['meshfile'] = gm.gen_stick(spos=np.array([0, 0, 0]),
                                                    epos=np.array([cylinder_height, 0, 0]),
                                                    thickness=link_width[2])
        self.jlc.lnks[2]['rgba'] = [.7, .7, .7, 0.5]
        # self.jlc.lnks[2]['com'] = np.array([.0, .0, .15])
        # self.jlc.lnks[2]['mass'] = 1.29

        self.jlc.lnks[3]['name'] = "j3"
        self.jlc.lnks[3]['loc_pos'] = np.array([link2_length, 0, link_width[3] / 2])
        self.jlc.lnks[3]['loc_rotmat'] = rm.rotmat_from_euler(0, math.pi / 2, 0)
        self.jlc.lnks[3]['meshfile'] = gm.gen_stick(spos=np.array([0, 0, 0]),
                                                    epos=np.array([link_width[3], 0, 0]),
                                                    thickness=link_width[3])
        self.jlc.lnks[3]['rgba'] = [.77, .77, .60, 0.5]
        # self.jlc.lnks[3]['com'] = np.array([-.02, .1, .07])
        # self.jlc.lnks[3]['mass'] = 0.39

        self.jlc.lnks[4]['name'] = "j4"
        self.jlc.lnks[4]['loc_pos'] = np.array([link_width[3] / 2, 0, 0])
        cylinder_height = link4_length - link_width[3] / 2 - link_width[5] / 2
        self.jlc.lnks[4]['meshfile'] = gm.gen_stick(spos=np.array([0, 0, 0]),
                                                    epos=np.array([cylinder_height, 0, 0]),
                                                    thickness=link_width[4])
        self.jlc.lnks[4]['rgba'] = [.35, .35, .35, 0.5]
        # self.jlc.lnks[4]['com'] = np.array([-.01, .02, .03])
        # self.jlc.lnks[4]['mass'] = .35

        self.jlc.lnks[5]['name'] = "j5"
        self.jlc.lnks[5]['loc_pos'] = np.array([link4_length, 0, link_width[5] / 2])
        self.jlc.lnks[5]['loc_rotmat'] = rm.rotmat_from_euler(0, math.pi / 2, 0)
        self.jlc.lnks[5]['meshfile'] = gm.gen_stick(spos=np.array([0, 0, 0]),
                                                    epos=np.array([link_width[5], 0, 0]),
                                                    thickness=link_width[5])
        self.jlc.lnks[5]['rgba'] = [.7, .7, .7, 0.5]
        # self.jlc.lnks[5]['com'] = np.array([.0, .0, 0.055])
        # self.jlc.lnks[5]['mass'] = 0.35

        self.jlc.lnks[6]['name'] = "j6"
        self.jlc.lnks[6]['loc_pos'] = np.array([link_width[5] / 2, 0, 0])
        cylinder_height = link6_length - link_width[5] / 2 - link_width[7] / 2
        self.jlc.lnks[6]['meshfile'] = gm.gen_stick(spos=np.array([0, 0, 0]),
                                                    epos=np.array([cylinder_height, 0, 0]),
                                                    thickness=link_width[6])
        self.jlc.lnks[6]['rgba'] = [.77, .77, .60, 0.5]
        # self.jlc.lnks[6]['com'] = np.array([.0, -.04, .015])
        # self.jlc.lnks[6]['mass'] = 0.19

        self.jlc.lnks[7]['name'] = "j7"
        self.jlc.lnks[7]['loc_pos'] = np.array([link6_length, 0, 0])
        self.jlc.lnks[7]['meshfile'] = gm.gen_box(extent=[link_width[7], link_width[7], link_width[7]])
        self.jlc.lnks[7]['rgba'] = [.5, .5, .5, 0.5]
        # self.jlc.lnks[7]['com'] = np.array([.0, .0, 0])
        # self.jlc.lnks[7]['mass'] = 0.03

        self.jlc.reinitialize()

        # collision detection
        if enable_cc:
            self.enable_cc()

    def enable_cc(self):
        super().enable_cc()
        # self.cc.add_cdlnks(self.jlc, [0, 1, 2, 3, 4, 5, 6, 7])
        # activelist = [self.jlc.lnks[0],
        #               self.jlc.lnks[1],
        #               self.jlc.lnks[2],
        #               self.jlc.lnks[3],
        #               self.jlc.lnks[4],
        #               self.jlc.lnks[5],
        #               self.jlc.lnks[6]]
        # self.cc.set_active_cdlnks(activelist)
        # fromlist = [self.jlc.lnks[0],
        #             self.jlc.lnks[1]]
        # intolist = [self.jlc.lnks[3],
        #             self.jlc.lnks[5],
        #             self.jlc.lnks[6]]
        # self.cc.set_cdpair(fromlist, intolist)
        # fromlist = [self.jlc.lnks[2]]
        # intolist = [self.jlc.lnks[4],
        #             self.jlc.lnks[5],
        #             self.jlc.lnks[6]]
        # self.cc.set_cdpair(fromlist, intolist)
        # fromlist = [self.jlc.lnks[3]]
        # intolist = [self.jlc.lnks[6]]
        # self.cc.set_cdpair(fromlist, intolist)


if __name__ == '__main__':
    import time
    import visualization.panda.world as wd
    import modeling.geometric_model as gm
    import modeling.collision_model as cm
    import utils

    base = wd.World(cam_pos=[-5, -5, 3], lookat_pos=[3, 0, 0])
    gm.gen_frame().attach_to(base)

    manipulator_instance = TBMArm(enable_cc=True)

    manipulator_meshmodel = manipulator_instance.gen_meshmodel()
    manipulator_meshmodel.attach_to(base)

    tbm_gate = utils.gen_gate(outer_radius=1.5, inner_radius=0.5, thickness=0.05, pos=np.array([4, 0, 0]),
                              pass_vec=np.array([1, 0, 0]), top_vec=np.array([0, 0, 1]))
    # tbm_gate.set_pos(np.array([4, 0, 0]))
    # tbm_gate.set_rotmat(rm.rotmat_from_euler(0, math.pi / 2, 0))
    tbm_gate.attach_to(base)

    # tbm_chamber = utils.gen_gate(body=1.5, passage=1.45, thickness=4, rgba=[0.4, 0, 0, 0.5])
    # tbm_chamber.set_pos(np.array([0, 0, 0]))
    # tbm_chamber.set_rotmat(rm.rotmat_from_euler(0, math.pi / 2, 0))
    # tbm_chamber.attach_to(base)

    jnt_values = manipulator_instance.get_jnt_values()
    print(jnt_values)
    # manipulator_instance.fk(jnt_values=jnt_values)

    base.run()
