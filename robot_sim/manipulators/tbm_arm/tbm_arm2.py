import os
import math
import numpy as np

import basis.robot_math as rm
import robot_sim._kinematics.jlchain as jl
import robot_sim.manipulators.manipulator_interface as mi


class TBMArm(mi.ManipulatorInterface):

    def __init__(self, pos=np.zeros(3), rotmat=np.eye(3), homeconf=np.zeros(6), name='tbm_arm', enable_cc=True):
        super().__init__(pos=pos, rotmat=rotmat, name=name)
        this_dir, this_filename = os.path.split(__file__)
        self.jlc = jl.JLChain(pos=pos, rotmat=rotmat, homeconf=homeconf, name=name)
        # six joints, njnts = 6+2 (tgt ranges from 1-6), nlinks = 6+1

        # self.jlc.jnts[1]['loc_pos'] = np.array([0.2, 0, 0])
        # self.jlc.jnts[1]['type'] = 'prismatic'
        # self.jlc.jnts[1]['loc_motionax'] = np.array([1, 0, 0])
        # self.jlc.jnts[1]['motion_rng'] = [-.1, 1.25]
        self.jlc.jnts[1]['loc_pos'] = np.array([0, 0, 0.346])
        self.jlc.jnts[1]['loc_motionax'] = np.array([0, 0, 1])
        self.jlc.jnts[1]['motion_rng'] = [-math.radians(30), math.radians(30)]
        # set joint 1 as pristmatic
        # self.jlc.jnts[1]['type'] = "prismatic"
        self.jlc.jnts[2]['loc_pos'] = np.array([0.645, .0, .0])
        self.jlc.jnts[2]['loc_motionax'] = np.array([1, 0, 0])
        # self.jlc.jnts[2]['motion_rng'] = [-2*math.pi, 2*math.pi]
        self.jlc.jnts[3]['loc_pos'] = np.array([.425, .0, .0])
        self.jlc.jnts[3]['loc_motionax'] = np.array([0, 0, 1])
        self.jlc.jnts[3]['motion_rng'] = [-math.radians(90), math.radians(90)]
        self.jlc.jnts[4]['loc_pos'] = np.array([0.587, .0, .0])
        self.jlc.jnts[4]['loc_motionax'] = np.array([1, 0, 0])
        # self.jlc.jnts[4]['motion_rng'] = [-2*math.pi, 2*math.pi]
        self.jlc.jnts[5]['loc_pos'] = np.array([.63, .0, .0])
        self.jlc.jnts[5]['loc_motionax'] = np.array([0, 1, 0])
        self.jlc.jnts[5]['motion_rng'] = [-math.radians(115), math.radians(115)]
        self.jlc.jnts[6]['loc_pos'] = np.array([.329, .0, .0])
        self.jlc.jnts[6]['loc_motionax'] = np.array([1, 0, 0])
        # self.jlc.jnts[6]['motion_rng'] = [-2*math.pi, 2*math.pi]

        # links
        self.jlc.lnks[0]['name'] = "base"
        self.jlc.lnks[0]['loc_pos'] = np.zeros(3)
        # self.jlc.lnks[0]['mass'] = 1.4
        # self.jlc.lnks[0]['com'] = np.array([-.02131, .000002, .044011])
        self.jlc.lnks[0]['meshfile'] = gm.gen_stick(spos=np.array([0, 0, 0]), epos=np.array([0, 0, 0.4]), thickness=.4)
        self.jlc.lnks[0]['rgba'] = [.5, .5, .5, 0.5]

        self.jlc.lnks[1]['name'] = "j1"
        self.jlc.lnks[1]['loc_pos'] = np.array([0.3, 0, 0])
        self.jlc.lnks[1]['loc_rotmat'] = rm.rotmat_from_euler(0, math.pi / 2, 0)
        self.jlc.lnks[1]['com'] = np.array([.0, .0, .15])
        self.jlc.lnks[1]['mass'] = 1.29
        self.jlc.lnks[1]['meshfile'] = gm.gen_stick(spos=np.array([0, 0, 0]), epos=np.array([0, 0, 0.4]), thickness=.4)
        self.jlc.lnks[1]['rgba'] = [.7, .7, .7, 0.5]

        self.jlc.lnks[2]['name'] = "j2"
        self.jlc.lnks[2]['loc_pos'] = np.array([.2, .0, -0.1])
        self.jlc.lnks[2]['com'] = np.array([-.02, .1, .07])
        self.jlc.lnks[2]['mass'] = 0.39
        self.jlc.lnks[2]['meshfile'] = gm.gen_stick(spos=np.array([0, 0, 0]), epos=np.array([0, 0, 0.2]), thickness=.3)
        self.jlc.lnks[2]['rgba'] = [.77, .77, .60, 0.5]

        self.jlc.lnks[3]['name'] = "j3"
        self.jlc.lnks[3]['loc_pos'] = np.array([.0, .0, .0])
        self.jlc.lnks[3]['com'] = np.array([-.01, .02, .03])
        self.jlc.lnks[3]['mass'] = .35
        self.jlc.lnks[3]['meshfile'] = os.path.join(this_dir, "meshes", "joint3.stl")
        self.jlc.lnks[3]['rgba'] = [.35, .35, .35, 1.0]
        self.jlc.lnks[4]['name'] = "j4"
        self.jlc.lnks[4]['loc_pos'] = np.array([.0, .0, .0])
        self.jlc.lnks[4]['com'] = np.array([.0, .0, 0.055])
        self.jlc.lnks[4]['mass'] = 0.35
        self.jlc.lnks[4]['meshfile'] = os.path.join(this_dir, "meshes", "joint4.stl")
        self.jlc.lnks[4]['rgba'] = [.7, .7, .7, 1.0]
        self.jlc.lnks[5]['name'] = "j5"
        self.jlc.lnks[5]['loc_pos'] = np.array([.0, .0, .0])
        self.jlc.lnks[5]['com'] = np.array([.0, -.04, .015])
        self.jlc.lnks[5]['mass'] = 0.19
        self.jlc.lnks[5]['meshfile'] = os.path.join(this_dir, "meshes", "joint5.stl")
        self.jlc.lnks[5]['rgba'] = [.77, .77, .60, 1]
        self.jlc.lnks[6]['name'] = "j6"
        self.jlc.lnks[6]['loc_pos'] = np.array([.0, .0, .0])
        self.jlc.lnks[6]['com'] = np.array([.0, .0, 0])
        self.jlc.lnks[6]['mass'] = 0.03
        self.jlc.lnks[6]['meshfile'] = None
        self.jlc.lnks[6]['rgba'] = [.5, .5, .5, 1.0]
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

    base = wd.World(cam_pos=[-5, -5, 3], lookat_pos=[3, 0, 0])
    gm.gen_frame().attach_to(base)

    manipulator_instance = TBMArm(enable_cc=True)
    manipulator_meshmodel = manipulator_instance.gen_meshmodel(toggle_jntscs=True)
    manipulator_meshmodel.attach_to(base)
    # manipulator_instance.gen_stickmodel(toggle_tcpcs=True, toggle_jntscs=True).attach_to(base)

    jnt_values = manipulator_instance.get_jnt_values()
    print(jnt_values)
    manipulator_instance.fk(jnt_values=jnt_values)

    manipulator_meshmodel = manipulator_instance.gen_meshmodel()
    manipulator_meshmodel.attach_to(base)

    robot_attached_list = []
    counter = [0]
    path = np.linspace(0, math.pi*2, 100)

    def update(manipulator_instance, path, robot_attached_list, counter, task):
        if counter[0] >= len(path):
            counter[0] = 0
        if len(robot_attached_list) != 0:
            for robot_attached in robot_attached_list:
                robot_attached.detach()
            robot_attached_list.clear()
        jnt_values[0] = path[counter[0]]
        jnt_values[1] = path[counter[0]]
        manipulator_instance.fk(jnt_values=jnt_values)
        robot_meshmodel = manipulator_instance.gen_meshmodel()
        robot_meshmodel.attach_to(base)
        robot_attached_list.append(robot_meshmodel)
        counter[0] += 1
        return task.again

    taskMgr.doMethodLater(0.01, update, "update",
                          extraArgs=[manipulator_instance, path, robot_attached_list, counter],
                          appendTask=True)

    base.run()
