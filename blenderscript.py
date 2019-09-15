import sys
sys.path.append('/home/lars/src/c++_to_python/')

import example

sp = example.surface_points()

import bpy

new_obj = bpy.data.objects.new('new_obj', bpy.data.objects['Cube'].data) 
bpy.data.collections[0].objects.link(new_obj)
