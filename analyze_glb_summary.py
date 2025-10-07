import sys
from pathlib import Path
import numpy as np
from pygltflib import GLTF2

COMPONENT_TYPE_TO_DTYPE = {
    5120: np.int8,
    5121: np.uint8,
    5122: np.int16,
    5123: np.uint16,
    5125: np.uint32,
    5126: np.float32,
}
TYPE_COMPONENT_COUNT = {
    "SCALAR": 1,
    "VEC2": 2,
    "VEC3": 3,
    "VEC4": 4,
    "MAT2": 4,
    "MAT3": 9,
    "MAT4": 16,
}

def get_accessor_data(gltf: GLTF2, accessor_index: int) -> np.ndarray:
    accessor = gltf.accessors[accessor_index]
    buffer_view = gltf.bufferViews[accessor.bufferView]
    buffer_data = gltf.binary_blob()
    component_dtype = COMPONENT_TYPE_TO_DTYPE[accessor.componentType]
    component_count = TYPE_COMPONENT_COUNT[accessor.type]
    byte_stride = buffer_view.byteStride or component_count * component_dtype().itemsize
    offset = (buffer_view.byteOffset or 0) + (accessor.byteOffset or 0)
    count = accessor.count
    data = buffer_data[offset: offset + byte_stride * count]
    array = np.frombuffer(data, dtype=component_dtype, count=count * (byte_stride // component_dtype().itemsize))
    array = array.reshape((count, byte_stride // component_dtype().itemsize))
    if byte_stride != component_count * component_dtype().itemsize:
        array = array[:, :component_count]
    return array.astype(np.float64)

def summarize(path: Path):
    gltf = GLTF2().load(str(path))
    mins = []
    maxs = []
    if not gltf.meshes:
        print(f"{path.name}: no meshes")
        return
    for mesh in gltf.meshes:
        for prim in mesh.primitives:
            pos_idx = prim.attributes.POSITION
            positions = get_accessor_data(gltf, pos_idx)
            mins.append(positions.min(axis=0))
            maxs.append(positions.max(axis=0))
    mins = np.min(np.array(mins), axis=0)
    maxs = np.max(np.array(maxs), axis=0)
    size = maxs - mins
    center = (maxs + mins) / 2
    axes = ['X','Y','Z']
    order = np.argsort(size)[::-1]
    print(f"{path.name}: size={size}, center={center}, dominant axes={(axes[order[0]], size[order[0]])}")

if __name__ == '__main__':
    for arg in sys.argv[1:]:
        summarize(Path(arg))
