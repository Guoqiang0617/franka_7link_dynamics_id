from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .inertial import inertial_origin_to_com, matrix_to_six, parallel_axis_matrix


def write_identified_urdf(
    source_urdf: Path,
    destination_urdf: Path,
    rigid_parameters,
    link_prefix: str = "fr3_link",
) -> Path:
    source_urdf = Path(source_urdf).resolve()
    destination_urdf = Path(destination_urdf).resolve()
    if source_urdf == destination_urdf:
        raise ValueError("The source URDF is never modified; choose a separate output path.")
    if not source_urdf.is_file():
        raise FileNotFoundError(source_urdf)
    parameters = np.asarray(rigid_parameters, dtype=float).reshape(7, 10)

    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(source_urdf, parser=parser)
    robot = tree.getroot()
    links = {link.get("name"): link for link in robot.findall("link")}
    updated = []
    for index, link_parameters in enumerate(parameters, start=1):
        link_name = f"{link_prefix}{index}"
        link = links.get(link_name)
        if link is None:
            raise ValueError(
                f"URDF has no link named {link_name!r}; set --link-prefix for this robot."
            )
        inertial = link.find("inertial")
        if inertial is None:
            raise ValueError(f"URDF link {link_name} has no <inertial> element.")
        mass_tag = inertial.find("mass")
        origin_tag = inertial.find("origin")
        inertia_tag = inertial.find("inertia")
        if mass_tag is None or origin_tag is None or inertia_tag is None:
            raise ValueError(f"URDF link {link_name} has an incomplete inertial element.")

        mass, com, inertia_com = inertial_origin_to_com(link_parameters)
        mass_tag.set("value", f"{mass:.12g}")
        origin_tag.set("xyz", " ".join(f"{value:.12g}" for value in com))
        origin_tag.set("rpy", "0 0 0")
        for key, value in (
            ("ixx", inertia_com[0, 0]),
            ("ixy", inertia_com[0, 1]),
            ("ixz", inertia_com[0, 2]),
            ("iyy", inertia_com[1, 1]),
            ("iyz", inertia_com[1, 2]),
            ("izz", inertia_com[2, 2]),
        ):
            inertia_tag.set(key, f"{value:.12g}")
        updated.append(link_name)

    destination_urdf.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(destination_urdf, encoding="utf-8", xml_declaration=True)
    print(f"Updated inertial elements in {len(updated)} links: {destination_urdf}")
    return destination_urdf


def read_urdf_rigid_parameters(path: Path, link_prefix="fr3_link") -> np.ndarray:
    """Read the exported zero-rpy inertial frames back into link-origin parameters."""
    links = {node.get("name"): node for node in ET.parse(path).getroot().findall("link")}
    values = []
    for index in range(1, 8):
        inertial = links[f"{link_prefix}{index}"].find("inertial")
        mass = float(inertial.find("mass").get("value"))
        origin = inertial.find("origin")
        com = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        if not np.allclose(np.fromstring(origin.get("rpy", "0 0 0"), sep=" "), 0.0):
            raise ValueError("This FR3 template/export reader requires zero inertial origin rpy.")
        tag = inertial.find("inertia")
        inertia_com = np.array([[float(tag.get("ixx")), float(tag.get("ixy")), float(tag.get("ixz"))],
                                [float(tag.get("ixy")), float(tag.get("iyy")), float(tag.get("iyz"))],
                                [float(tag.get("ixz")), float(tag.get("iyz")), float(tag.get("izz"))]])
        inertia_origin = inertia_com + mass * parallel_axis_matrix(com)
        parameter = np.r_[mass, mass * com, matrix_to_six(inertia_origin)]
        inertial_origin_to_com(parameter)
        values.extend(parameter)
    return np.asarray(values)


def export_models(parameter_file, config, source_urdf, destination_urdf, mjcf_dir, link_prefix="fr3_link"):
    from .simulation import write_mjcf
    from .parameter_io import load_parameter_vectors

    prior, identified = load_parameter_vectors(parameter_file, config)
    nominal_from_urdf = read_urdf_rigid_parameters(source_urdf, link_prefix)
    if not np.allclose(nominal_from_urdf, prior[:70], atol=1e-9, rtol=1e-7):
        raise ValueError("The URDF template does not match the nominal prior used for identification.")
    write_identified_urdf(source_urdf, destination_urdf, identified[:70], link_prefix)
    identified_from_urdf = read_urdf_rigid_parameters(destination_urdf, link_prefix)
    if not np.allclose(identified_from_urdf, identified[:70], atol=1e-9, rtol=1e-7):
        raise ValueError("URDF export round trip changed the identified inertial parameters.")
    directory = Path(mjcf_dir)
    nominal = write_mjcf(directory / "fr3_ffs_nominal.xml", nominal_from_urdf, config, "fr3_ffs_nominal")
    fitted = write_mjcf(directory / "fr3_ffs_identified.xml", identified_from_urdf, config, "fr3_ffs_identified")
    return nominal, fitted
