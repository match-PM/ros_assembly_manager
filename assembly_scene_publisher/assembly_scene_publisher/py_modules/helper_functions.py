from graphviz import Digraph

from ament_index_python import get_package_share_directory

def extract_graph(tree: dict):
    """
    Convert nested dictionary tree into
    a set of unique nodes and edges.
    """
    nodes = set()
    edges = set()

    def traverse(parent, subtree):
        nodes.add(parent)

        for child, child_subtree in subtree.items():
            nodes.add(child)
            edges.add((parent, child))
            traverse(child, child_subtree)

    for root, subtree in tree.items():
        traverse(root, subtree)

    return nodes, edges


def render_graph(nodes, edges, output_filename=None):
    path = get_package_share_directory("assembly_scene_publisher")
    dot = Digraph(comment="Reference Tree")
    output_filename = f"{path}/image_renders/{output_filename}" if output_filename else None

    dot.attr(rankdir="TB")
    dot.attr("node", shape="box")
    dot.attr(dpi="200")

    for node in nodes:
        dot.node(node)

    for parent, child in edges:
        dot.edge(parent, child)

    if output_filename:
        dot.render(output_filename, format="png", cleanup=True)

    # Return PNG as bytes
    return dot.pipe(format="png")

def strip_substring_from_keys(d, substring):
    """
    Recursively remove a substring from all keys in a nested dict.
    """
    if not isinstance(d, dict):
        return d  # Base case: not a dict, return as is

    new_dict = {}
    for key, value in d.items():
        # Remove substring from key
        new_key = key.replace(substring, "")
        # Recurse into value if it's a dict
        new_dict[new_key] = strip_substring_from_keys(value, substring)
    return new_dict