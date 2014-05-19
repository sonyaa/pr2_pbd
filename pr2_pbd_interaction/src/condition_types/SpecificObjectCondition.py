#!/usr/bin/env python
import rospy
from World import World
from condition_types.Condition import Condition


class SpecificObjectCondition(Condition):
    '''
    Checks that the objects present are similar enough to the specified ones.
    '''

    def __init__(self, *args, **kwargs):
        Condition.__init__(self, *args, **kwargs)
        self.objects = []

    def add_object(self, obj):
        self.objects.append(obj)

    def delete_last_object(self):
        self.objects.pop()

    def is_empty(self):
        return len(self.objects) == 0

    def get_objects(self):
        return self.objects

    def get_unique_objects(self):
        objects = []
        object_names = []
        for obj in self.objects:
            if obj is not None and obj.name not in object_names:
                object_names.append(obj.name)
                objects.append(obj)
        return objects

    def check(self):
        # look at the state of the world, verify that the world objects are similar to ours
        if self.is_empty():
            return True
        if len(self.get_unique_objects()) == 0:
            return True
        world_objects = World.get_world().get_frame_list()
        map_of_objects_old_to_new = World.get_map_of_most_similar_obj(self.get_unique_objects(), world_objects)
        if map_of_objects_old_to_new is None:
            # didn't find similar objects
            rospy.logwarn("Condition failure: didn't find similar objects")
            return False
        return True

    def __repr__(self):
        return "%s(objects=%r)" % (
            self.__class__.__name__, self.objects)