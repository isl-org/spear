import argparse
import json
import os
import pandas as pd
import posixpath
import unreal
import spear


parser = argparse.ArgumentParser()
parser.add_argument("--pipeline_dir", required=True)
args = parser.parse_args()

unreal_paths = unreal.Paths()
editor_properties_csv_file = os.path.realpath(os.path.join(unreal_paths.project_content_dir(), "Python", "editor_properties.csv"))
df_editor_properties = pd.read_csv(editor_properties_csv_file)


def process_scene():
    global args

    editor_actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    unreal_editor_subsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)

    editor_world_name = unreal_editor_subsystem.get_editor_world().get_name()

    actors = editor_actor_subsystem.get_all_level_actors()
    actors = sorted(actors, key=lambda actor: get_debug_string_actor(actor))
    actor_descs = { get_debug_string_actor(actor): get_actor_desc(actor) for actor in actors }

    metadata_dir = os.path.realpath(os.path.join(args.pipeline_dir, editor_world_name, "scene_metadata"))
    metadata_json_file = os.path.realpath(os.path.join(metadata_dir, "outliner.json"))

    os.makedirs(metadata_dir, exist_ok=True)
    with open(metadata_json_file, "w") as f:
        json.dump(actor_descs, f, indent=4, sort_keys=True)

    spear.log("Done.")


def get_actor_desc(actor):

    actor_class = actor.__class__.__name__

    # Ignore root_component when getting all of the editor properties for this actor, because we will
    # get the actor's component hierarchy separately.
    editor_property_descs = get_editor_property_descs(actor, ignore=["root_component"])

    # It is possible for an actor not to have a root component.
    root_component = actor.get_editor_property("root_component")
    if root_component is None:
        root_component_desc = None
    else:
        root_component_desc = {get_debug_string_component(root_component): get_component_desc(root_component)}

    return {"class": actor_class, "editor_properties": editor_property_descs, "root_component": root_component_desc}


def get_component_desc(component):
    component_class = component.__class__.__name__
    editor_property_descs = get_editor_property_descs(component)
    children_component_descs = { get_debug_string_component(c): get_component_desc(c) for c in component.get_children_components(include_all_descendants=False) }
    return {"class": component_class, "editor_properties": editor_property_descs, "children_components": children_component_descs}


def get_editor_property_descs(uobject, ignore=[]):

    assert "get_editor_property" in dir(uobject)

    # The Unreal Python interface does not provide a mechanism to iterate over editor properties directly,
    # so we use the fact that most editor properties are exposed as Python attributes, and iterate over
    # attributes instead. For each attribute, we use a guess-and-check strategy to see if it is an editor
    # property. We call obj.get_editor_property(...) and if doing so doesn't throw an exception, we assume
    # that the attribute is a valid editor property. For each editor property that we find, if the value
    # of the editor property is itself a uobject, then we recurse, otherwise we store its value in our
    # output dict. To obtain the remaining editor properties that are not exposed as Python attributes, we
    # maintain a CSV file with a list of editor properties for various classes of interest. We populate
    # the CSV file by manually copying and pasting from the Unreal documentation.

    editor_property_descs = {}

    # Guess-and-check all Python attributes.
    candidate_editor_property_names = set(dir(uobject))
    for candidate_editor_property_name in candidate_editor_property_names:
        is_editor_property = False
        try:
            editor_property = uobject.get_editor_property(candidate_editor_property_name)
            is_editor_property = True
        except:
            pass
        if is_editor_property:
            editor_property_descs[candidate_editor_property_name] = get_editor_property_desc(editor_property)

    # Get all editor properties in our CSV file that match the uobjects's class and base classes.
    uobject_classes = [uobject.__class__.__name__] + [ base_class.__name__ for base_class in uobject.__class__.__bases__ ][::-1]
    editor_property_names = set()
    for uobject_class in uobject_classes:
        editor_property_names = editor_property_names | set(df_editor_properties.loc[df_editor_properties["class"] == uobject_class]["editor_property"])

    # Remove Python attributes because we checked them already.
    editor_property_names = editor_property_names.difference(candidate_editor_property_names)

    # Get editor property descs.
    for editor_property_name in editor_property_names:
        editor_property = uobject.get_editor_property(editor_property_name)
        editor_property_descs[editor_property_name] = get_editor_property_desc(editor_property)

    return editor_property_descs


def get_editor_property_desc(editor_property):

    editor_property_class = editor_property.__class__.__name__

    # If the editor property is an Actor, then do not return any editor properties to avoid an infinite
    # recursion. If users want to obtain the editor properties for an Actor, they must call
    # get_editor_property_descs(...).
    if isinstance(editor_property, unreal.Actor):
        actor_name = get_debug_string_actor(editor_property)
        return {"class": editor_property_class, "editor_properties": "...", "name": actor_name}

    # Otherwise, if the editor property is an ActorComponent, then do not return any editor properties
    # to avoid an infinite recursion. If users want to obtain the editor properties for an Actor, they
    # must call get_editor_property_descs(...).
    elif isinstance(editor_property, unreal.ActorComponent):
        component_name = get_debug_string_component(editor_property)        
        return {"class": editor_property_class, "editor_properties": "...", "name": component_name}

    # Otherwise, if the editor property is an Unreal object, then recurse via get_editor_property_descs(...)
    # and return a dict of the form {"class": ..., "editor_properties": ...}.
    elif isinstance(editor_property, unreal.Object):
        editor_property_descs = get_editor_property_descs(editor_property)
        return {"class": editor_property_class, "editor_properties": editor_property_descs}

    # Otherwise, if the editor property is an Unreal struct, then recurse via get_editor_property_descs(...)
    # and return a dict of the form {"class": ..., "editor_properties": ...}.
    elif isinstance(editor_property, unreal.StructBase):
        editor_property_descs = get_editor_property_descs(editor_property)
        return {"class": editor_property_class, "editor_properties": editor_property_descs}

    # Otherwise, if the editor property is an Unreal object, then recurse via get_editor_property_desc(...)
    # and return a list.
    elif isinstance(editor_property, unreal.Array):
        return [ get_editor_property_desc(editor_property_array_entry) for editor_property_array_entry in editor_property ]

    # Otherwise, if the editor property value if serializable as JSON, then return the object, otherwise
    # return the string representation of the object.
    else:
        try:
            json.dumps(editor_property)
            return editor_property # if the value if serializable as JSON, then return the object
        except:
            return str(editor_property) # otherwise return the string representation of the object


def get_debug_string_actor(actor):
    folder_path = actor.get_folder_path()
    if folder_path.is_none():
        return actor.get_actor_label()
    else:
        return str(folder_path) + posixpath.sep + actor.get_actor_label()


def get_debug_string_component(component):
    parent_components = list(component.get_parent_components())[::-1]
    if len(parent_components) == 0:
        return component.get_name()
    else:
        return ".".join([ parent_component.get_name() for parent_component in parent_components ]) + "." + component.get_name()


if __name__ == "__main__":
    process_scene()
