import bpy
import math

from importlib import reload

import FourierBlender_helper
import FourierBlender_interface

reload(FourierBlender_helper)
reload(FourierBlender_interface)

# clean up from previous runs
for obj in bpy.data.objects:
    if obj.users == 0:
        bpy.data.objects.remove(obj)
for action in bpy.data.actions:
    # somehow there are always users...
    #if action.users == 0:
    #    bpy.data.actions.remove(action)
    bpy.data.actions.remove(action)
for mesh in bpy.data.meshes:
    if mesh.users == 0:
        bpy.data.meshes.remove(mesh)

template_mesh = bpy.data.meshes["template_mesh"]

num_obj = 10
num_frames = 100

fourierBlender = FourierBlender_interface.FourierBlender(num_obj, num_frames)

for i in range(0, num_obj):
    FourierBlender_helper.addArrow(num_frames, template_mesh, str(i),\
            fourierBlender.scales(i),\
            fourierBlender.locations(i),\
            fourierBlender.rotations(i))
