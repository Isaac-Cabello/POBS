import xml.etree.ElementTree as ET


def read_pose(element):
    # SDF poses contain: x, y, z, roll, pitch, yaw.
    # If no pose is given, it defaults to all zeros.
    text = element.findtext("pose", "0 0 0 0 0 0")
    return tuple(float(value) for value in text.split())


def load_model_properties(model_path, world_path):
    model = ET.parse(model_path).getroot().find("model")
    world = ET.parse(world_path).getroot().find("world")

    links = []

    for link in model.findall("link"):
        inertial = link.find("inertial")
        collisions = []

        for collision in link.findall("collision"):
            # Your current model uses box collision geometry.
            size = collision.findtext("geometry/box/size")
            if size is None:
                raise ValueError("Only box collisions are supported.")

            collisions.append({
                "pose": read_pose(collision),
                "size": tuple(float(value) for value in size.split()),
            })

        links.append({
            "name": link.get("name"),
            "pose": read_pose(link),
            "mass": float(inertial.findtext("mass")),
            "inertial_pose": read_pose(inertial),
            "collisions": collisions,
        })

    buoyancy = world.find(
        "plugin[@name='gz::sim::systems::Buoyancy']"
    )

    water_density = float(
        buoyancy.findtext("graded_buoyancy/default_density")
    )

    gravity = tuple(
        float(value) for value in world.findtext("gravity").split()
    )

    return {
        "links": links,
        "water_density": water_density,
        "gravity": gravity,
    }