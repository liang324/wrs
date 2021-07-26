from .generic import MeshScript
from ..templates import get_template
from distutils.spawn import find_executable

_blender_executable = find_executable('blender', r'C:\Program Files\Blender Foundation\Blender')  # 20210718, duke
_blender_template = get_template('blender.py.template')
exists = _blender_executable is not None


# def boolean(meshes, operation='difference'):
#     if not exists:
#         raise ValueError('No blender available!')
#     operation = str.upper(operation)
#     if operation == 'INTERSECTION':
#         operation = 'INTERSECT'
#     script = _blender_template.replace('$operation', operation)
#     with MeshScript(meshes=meshes,
#                     script=script) as blend:
#         result = blend.run(_blender_executable + ' --background --python $script')
#     # blender returns actively incorrect face normals
#     result['face_normals'] = None
#     return result


# use the lastest template
# 20210718, duke
from .. import util
from .. import resources
def boolean(meshes, operation='difference', debug=False):
    """
    Run a boolean operation with multiple meshes using Blender.
    """
    if not exists:
        raise ValueError('No blender available!')
    operation = str.upper(operation)
    if operation == 'INTERSECTION':
        operation = 'INTERSECT'

    # get the template from our resources folder
    template = resources.get('blender_boolean.py.template')
    script = template.replace('$OPERATION', operation)

    with MeshScript(meshes=meshes,
                    script=script,
                    debug=debug) as blend:
        result = blend.run(_blender_executable +
                           ' --background --python $SCRIPT')

    for m in util.make_sequence(result):
        # blender returns actively incorrect face normals
        m.face_normals = None

    return result