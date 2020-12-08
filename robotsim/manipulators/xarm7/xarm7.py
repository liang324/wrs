import osimport mathimport numpy as npimport basis.robotmath as rmimport robotsim._kinematics.jlchain as jlfrom panda3d.core import NodePath, CollisionTraverser, CollisionHandlerQueue, BitMask32class XArm7(jl.JLChain):    def __init__(self, pos=np.zeros(3), rotmat=np.eye(3), homeconf=np.zeros(7), name='xarm7'):        super().__init__(pos=pos, rotmat=rotmat, homeconf=homeconf, name=name)        this_dir, this_filename = os.path.split(__file__)        # seven joints, njnts = 7+2 (tgt ranges from 1-7), nlinks = 7+1        self.jnts[1]['loc_pos'] = np.array([0, 0, .267])        self.jnts[1]['rngmin'] = -math.pi        self.jnts[1]['rngmax'] = math.pi        self.jnts[2]['loc_pos'] = np.array([0, 0, 0])        self.jnts[2]['loc_rotmat'] = rm.rotmat_from_euler(-1.5708, 0, 0)        self.jnts[2]['rngmin'] = -2.18        self.jnts[2]['rngmax'] = 2.18        self.jnts[3]['loc_pos'] = np.array([0, -.293, 0])        self.jnts[3]['loc_rotmat'] = rm.rotmat_from_euler(1.5708, 0, 0)        self.jnts[3]['rngmin'] = -math.pi        self.jnts[3]['rngmax'] = math.pi        self.jnts[4]['loc_pos'] = np.array([.0525, 0, 0])        self.jnts[4]['loc_rotmat'] = rm.rotmat_from_euler(1.5708, 0, 0)        self.jnts[4]['rngmin'] = -0.11        self.jnts[4]['rngmax'] = math.pi        self.jnts[5]['loc_pos'] = np.array([0.0775, -0.3425, 0])        self.jnts[5]['loc_rotmat'] = rm.rotmat_from_euler(1.5708, 0, 0)        self.jnts[5]['rngmin'] = -math.pi        self.jnts[5]['rngmax'] = math.pi        self.jnts[6]['loc_pos'] = np.array([0, 0, 0])        self.jnts[6]['loc_rotmat'] = rm.rotmat_from_euler(1.5708, 0, 0)        self.jnts[6]['rngmin'] = -1.75        self.jnts[6]['rngmax'] = math.pi        self.jnts[7]['loc_pos'] = np.array([0.076, 0.097, 0])        self.jnts[7]['loc_rotmat'] = rm.rotmat_from_euler(-1.5708, 0, 0)        self.jnts[7]['rngmin'] = -math.pi        self.jnts[7]['rngmax'] = math.pi        # links        self.lnks[0]['name'] = "link_base"        self.lnks[0]['loc_pos'] = np.zeros(3)        self.lnks[0]['com'] = np.array([-0.021131, -0.0016302, 0.056488])        self.lnks[0]['mass'] = 0.88556        self.lnks[0]['meshfile'] = os.path.join(this_dir, "meshes", "link_base.stl")        self.lnks[0]['rgba'] = [.5,.5,.5,1.0]        self.lnks[1]['name'] = "link1"        self.lnks[1]['loc_pos'] = np.zeros(3)        self.lnks[1]['com'] = np.array([-0.0042142, 0.02821, -0.0087788])        self.lnks[1]['mass'] = 0.42603        self.lnks[1]['meshfile'] = os.path.join(this_dir, "meshes", "link1.stl")        self.lnks[2]['name'] = "link2"        self.lnks[2]['loc_pos'] = np.zeros(3)        self.lnks[2]['com'] = np.array([-3.3178e-5, -0.12849, 0.026337])        self.lnks[2]['mass'] = 0.56095        self.lnks[2]['meshfile'] = os.path.join(this_dir, "meshes", "link2.stl")        self.lnks[2]['rgba'] = [.5,.5,.5,1.0]        self.lnks[3]['name'] = "link3"        self.lnks[3]['loc_pos'] = np.zeros(3)        self.lnks[3]['com'] = np.array([0.04223, -0.023258, -0.0096674])        self.lnks[3]['mass'] = 0.44463        self.lnks[3]['meshfile'] = os.path.join(this_dir, "meshes", "link3.stl")        self.lnks[4]['name'] = "link4"        self.lnks[4]['loc_pos'] = np.zeros(3)        self.lnks[4]['com'] = np.array([0.067148, -0.10732, 0.024479])        self.lnks[4]['mass'] = 0.52387        self.lnks[4]['meshfile'] = os.path.join(this_dir, "meshes", "link4.stl")        self.lnks[4]['rgba'] = [.3,.5,.3,1.0]        self.lnks[5]['name'] = "link5"        self.lnks[5]['loc_pos'] = np.zeros(3)        self.lnks[5]['com'] = np.array([-0.00023397, 0.036705, -0.080064])        self.lnks[5]['mass'] = 0.18554        self.lnks[5]['meshfile'] = os.path.join(this_dir, "meshes", "link5.stl")        self.lnks[6]['name'] = "link6"        self.lnks[6]['loc_pos'] = np.zeros(3)        self.lnks[6]['com'] = np.array([0.058911, 0.028469, 0.0068428])        self.lnks[6]['mass'] = 0.31344        self.lnks[6]['meshfile'] = os.path.join(this_dir, "meshes", "link6.stl")        self.lnks[6]['rgba'] = [.5,.5,.5,1.0]        self.lnks[7]['name'] = "link7"        self.lnks[7]['loc_pos'] = np.zeros(3)        self.lnks[7]['com'] = np.array([-1.5846e-5, -0.0046377, -0.012705])        self.lnks[7]['mass'] = 0.31468        self.lnks[7]['meshfile'] = os.path.join(this_dir, "meshes", "link7.stl")        # reinitialization        # self.setinitvalues(np.array([-math.pi/2, math.pi/3, math.pi/6, 0, 0, 0, 0]))        # self.setinitvalues(np.array([-math.pi/2, 0, math.pi/3, math.pi/10, 0, 0, 0]))        self.reinitialize()        # pairs for collision detection        self.linkcdpairs = [[0, 1, 2], [5, 6, 7]]    def is_selfcollided(self, toggleplot=False):        oocnp = NodePath("collision nodepath")        obj0cnplist = []        for onelcdp in self.linkcdpairs[0]:            this_collisionmodel = self.lnks[onelcdp]['collisionmodel']            pos = self.lnks[onelcdp]['gl_pos']            rotmat = self.lnks[onelcdp]['gl_rotmat']            obj0cnplist.append(this_collisionmodel.copy_cdnp_to(oocnp, rm.homomat_from_posrot(pos, rotmat)))        obj1cnplist = []        for onelcdp in self.linkcdpairs[1]:            this_collisionmodel = self.lnks[onelcdp]['collisionmodel']            pos = self.lnks[onelcdp]['gl_pos']            rotmat = self.lnks[onelcdp]['gl_rotmat']            obj1cnplist.append(this_collisionmodel.copy_cdnp_to(oocnp, rm.homomat_from_posrot(pos, rotmat)))        if toggleplot:            oocnp.reparentTo(base.render)            for obj0cnp in obj0cnplist:                obj0cnp.show()            for obj1cnp in obj1cnplist:                obj1cnp.show()        ctrav = CollisionTraverser()        chan = CollisionHandlerQueue()        for obj0cnp in obj0cnplist:            obj0cnp.node().setFromCollideMask(BitMask32(0x1))            obj0cnp.setCollideMask(BitMask32(0x2))            ctrav.addCollider(obj0cnp, chan)        ctrav.traverse(oocnp)        if chan.getNumEntries() > 0:            return True        else:            return False    # def is_selfcollided2(self): # DEPRECATED: costs around 0.002s 20201115    #     cmlist0 = []    #     for onelcdp in self.linkcdpairs[0]:    #         this_collisionmodel = self.lnks[onelcdp]['collisionmodel'].copy()    #         pos = self.lnks[onelcdp]['gl_pos']    #         rotmat = self.lnks[onelcdp]['gl_rotmat']    #         this_collisionmodel.sethomomat(rm.homomat_from_posrot(pos, rotmat))    #         cmlist0.append(this_collisionmodel)    #     cmlist1 = []    #     for onelcdp in self.linkcdpairs[1]:    #         this_collisionmodel = self.lnks[onelcdp]['collisionmodel'].copy()    #         pos = self.lnks[onelcdp]['gl_pos']    #         rotmat = self.lnks[onelcdp]['gl_rotmat']    #         this_collisionmodel.sethomomat(rm.homomat_from_posrot(pos, rotmat))    #         cmlist1.append(this_collisionmodel)    #     return pcd.is_cmlistcmlist_collided(cmlist0, cmlist1)if __name__ == '__main__':    import time    import visualization.panda.world as wd    import modeling.geometricmodel as gm    base = wd.World(campos=[2, 0, 1], lookatpos=[0, 0, 0.5])    gm.gen_frame().attach_to(base)    manipulator_instance = XArm7()    manipulator_instance.gen_meshmodel().attach_to(base)    manipulator_instance.gen_stickmodel().attach_to(base)    tic = time.time()    print(manipulator_instance.is_selfcollided(toggleplot=True))    toc = time.time()    print(toc - tic)    base.run()