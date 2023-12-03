import json
import numpy as np

def adjust_coordinates(side_length=.04, offset=0, origin=[0,-0.08,0], filename="model.json"):
    """
    Adjust the block coordinates based on side length, offset and new origin.

    Parameters:
    - filename: The name of the JSON file to read from.
    - side_length: The length of a side of each block.
    - offset: The offset distance between blocks.
    - origin: A tuple representing the new (x, y, z) origin.

    Returns:
    - List of adjusted (x, y, z) tuples.
    """
    with open(filename, 'r') as f:
        blocks, = json.load(f).values()

    adjusted_blocks = []
    for block in blocks:
        adjusted_x = origin[0] + block[0] * (side_length + offset)
        adjusted_y = origin[1] + block[1] * (side_length + offset)
        adjusted_z = origin[2] + block[2] * (side_length + offset)
        adjusted_blocks.append((adjusted_x, adjusted_y, adjusted_z))

    # sort by z, then y, then x
    # adjusted_blocks.sort(key=lambda block: (block[2], block[1], block[0]))

    # return as numpy array
    adjusted_blocks = np.array(adjusted_blocks)

    # flip z and y 
    adjusted_blocks[:, [1, 2]] = adjusted_blocks[:, [2, 1]]

    return adjusted_blocks

print(adjust_coordinates())

