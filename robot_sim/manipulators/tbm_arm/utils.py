import math
import numpy as np
import basis.robot_math as rm
import modeling.collision_model as cm
import modeling.model_collection as mc


def gen_gate(sec=8,
             inner_radius=.8,
             outer_radius=1.5,
             thickness=.2,
             rgba=[1, 0, 0, 1],
             pass_vec=np.array([0, 0, 1]),
             top_vec=np.array([0, 1, 0]),
             pos=np.array([0, 0, 0]),
             name='gate'):
    side_vec = np.cross(top_vec, pass_vec)
    rotmat = np.eye(3)
    rotmat[:3, 0] = side_vec
    rotmat[:3, 1] = top_vec
    rotmat[:3, 2] = pass_vec
    mm_collection = mc.ModelCollection(name=name)
    for i in range(sec):
        mid_radius = (inner_radius + outer_radius) / 2
        mid_radius_projected = mid_radius * math.cos(math.pi / sec)
        center = [mid_radius_projected * math.sin(math.pi * 2 / sec * (i + .5)),
                  mid_radius_projected * math.cos(math.pi * 2 / sec * (i + .5)), 0]
        homomat = np.eye(4)
        homomat[:3, :3] = rm.rotmat_from_axangle(-np.array([0, 0, 1]), math.pi * 2 / sec * (i + .5))
        homomat[:3, 3] = center
        one_bcm = cm.gen_box(extent=[outer_radius * math.sin(math.pi / sec) * 2,
                                     (outer_radius - inner_radius) * math.cos(math.pi / sec),
                                     thickness],
                             homomat=homomat, rgba=rgba)
        one_bcm.set_rotmat(rotmat)
        one_bcm.set_pos(pos)
        one_bcm.attach_to(mm_collection)
    return mm_collection

if __name__ == '__main__':
    import visualization.panda.world as wd
    import modeling.geometric_model as gm
    import modeling.collision_model as cm

    base = wd.World(cam_pos=[0, 0, 3], lookat_pos=[0, 0, 0])
    gm.gen_frame().attach_to(base)
    gen_gate(pass_vec=np.array([1, 0, 0]), top_vec=np.array([0, 1, 0])).attach_to(base)
    base.run()
