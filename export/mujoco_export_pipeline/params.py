from dataclasses import dataclass
import enum
import shutil


# Find the v4.0 executable in the system path.
# Note trimesh has not updated their code to work with v4.0 which is why we do not use
# their `convex_decomposition` function.
VHACD_EXECUTABLE = shutil.which("TestVHACD")


# Names of the V-HACD output files.
VHACD_OUTPUTS = ["decomp.obj", "decomp.stl", "decomp.mtl"]


class FillMode(enum.Enum):
    FLOOD = enum.auto()
    SURFACE = enum.auto()
    RAYCAST = enum.auto()


@dataclass
class AssetProperties:
    name: str = ""
    id: int = 0
    moving: bool = False
    max_hulls: int = 0
    voxel_res: int = 0
    hull_vert_count: int = 0


@dataclass
class VhacdArgs:
    enable: bool = True
    """enable convex decomposition using V-HACD"""
    max_output_convex_hulls: int = 8
    """maximum number of output convex hulls for a single mesh"""
    voxel_resolution: int = 400000
    """total number of voxels to use (higher voxel resolutions will capture finer details but be more computationally intensive)"""
    volume_error_percent: float = 0.001
    """volume error allowed as a percentage (lower values will result in higher number of convex hulls. Allow detecting regions that are already convex and stop spend time on them)"""
    max_recursion_depth: int = 64
    """maximum recursion depth"""
    disable_shrink_wrap: bool = False
    """do not shrink wrap output to source mesh to better match the original mesh"""
    fill_mode: FillMode = FillMode.FLOOD
    """fill mode"""
    max_hull_vert_count: int = 512
    """maximum number of vertices in the output convex hull (the higher the greater the computation effort on the physics engine side)"""
    disable_async: bool = False
    """do not run asynchronously"""
    min_edge_length: int = 2
    """minimum size of a voxel edge (default is usually fine)"""
    split_hull: bool = False
    """try to find optimal split plane location (experimental)"""


@dataclass
class CoacdArgs:
    auto_threshold: bool = True
    """Whether threshold regression is used"""
    threshold: float = 0.02
    """Termination criteria in [0.01, 1] (0.01: most fine-grained; 1: most coarse), used only if auto_threshold == False"""
    threshold_model: tuple = (-0.24555493, 0.23217532)
    """model for regressing threshold, used only if auto_threshold == True"""
    max_convex_hull: int = -1
    """Maximum number of convex hulls in the result, -1 for no limit, works only when merge is enabled."""
    preprocess_mode: str = "auto"
    """No remeshing before running CoACD. Only suitable for manifold input."""
    preprocess_resolution: int = 100
    """Preprocessing resolution."""
    resolution: int = 2000
    """Surface samping resolution for Hausdorff distance computation."""
    mcts_nodes: int = 40
    """Number of cut candidates for MCTS."""
    mcts_iterations: int = 500
    """Number of MCTS iterations."""
    mcts_max_depth: int = 7
    """Maximum depth for MCTS search."""
    pca: bool = False
    """Use PCA to align input mesh. Suitable for non-axis-aligned mesh."""
    merge: bool = True
    """If merge is enabled, try to reduce total number of parts by merging."""
    seed: int = 0
    """Random seed."""


@dataclass
class ACD_Args:
    decompose_method : str
    coacd_args: CoacdArgs
    vhacd_args: VhacdArgs
    min_volume: float = 1e-3