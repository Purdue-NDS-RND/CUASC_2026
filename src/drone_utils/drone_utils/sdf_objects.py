import subprocess
import logging

_logger = logging.getLogger(__name__)


def _build_box_sdf(model_name: str, size, rgba) -> str:
    sx, sy, sz = size
    r, g, b, a = rgba
    return (
        "<sdf version='1.7'>"
        f"<model name='{model_name}'>"
        "<static>true</static>"
        "<link name='link'>"
        "<collision name='collision'>"
        "<geometry>"
        f"<box><size>{sx} {sy} {sz}</size></box>"
        "</geometry>"
        "</collision>"
        "<visual name='visual'>"
        "<geometry>"
        f"<box><size>{sx} {sy} {sz}</size></box>"
        "</geometry>"
        "<material>"
        "<ambient>"
        f"{r} {g} {b} {a}"
        "</ambient>"
        "<diffuse>"
        f"{r} {g} {b} {a}"
        "</diffuse>"
        "</material>"
        "</visual>"
        "</link>"
        "</model>"
        "</sdf>"
    )


def _build_digit_sdf(digit: int, cx: float, cy: float, size: float, z: float) -> str:
    """Build SDF elements for a 7-segment style digit.
    
    Segments layout:
       AAA
      F   B
       GGG
      E   C
       DDD
    
    Args:
        digit: 0-9
        cx, cy: center position
        size: overall height of digit
        z: z position
    """
    # Segment dimensions
    seg_len = size * 0.4   # Horizontal segment length
    seg_w = size * 0.12    # Segment width/thickness
    v_len = size * 0.35    # Vertical segment length
    
    # Positions relative to center
    top_y = cy + size * 0.35
    mid_y = cy
    bot_y = cy - size * 0.35
    left_x = cx - seg_len * 0.4
    right_x = cx + seg_len * 0.4
    
    # Which segments are on for each digit (A,B,C,D,E,F,G)
    segments = {
        0: [1,1,1,1,1,1,0],
        1: [0,1,1,0,0,0,0],
        2: [1,1,0,1,1,0,1],
        3: [1,1,1,1,0,0,1],
        4: [0,1,1,0,0,1,1],
        5: [1,0,1,1,0,1,1],
        6: [1,0,1,1,1,1,1],
        7: [1,1,1,0,0,0,0],
        8: [1,1,1,1,1,1,1],
        9: [1,1,1,1,0,1,1],
    }
    
    segs = segments.get(digit, segments[0])
    result = ""
    mat = "<material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>"
    
    # A - top horizontal
    if segs[0]:
        result += (
            f"<link name='seg_a'><pose>{cx:.4f} {top_y:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_len:.4f} {seg_w:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # B - top right vertical
    if segs[1]:
        result += (
            f"<link name='seg_b'><pose>{right_x:.4f} {(top_y+mid_y)/2:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_w:.4f} {v_len:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # C - bottom right vertical
    if segs[2]:
        result += (
            f"<link name='seg_c'><pose>{right_x:.4f} {(mid_y+bot_y)/2:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_w:.4f} {v_len:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # D - bottom horizontal
    if segs[3]:
        result += (
            f"<link name='seg_d'><pose>{cx:.4f} {bot_y:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_len:.4f} {seg_w:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # E - bottom left vertical
    if segs[4]:
        result += (
            f"<link name='seg_e'><pose>{left_x:.4f} {(mid_y+bot_y)/2:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_w:.4f} {v_len:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # F - top left vertical
    if segs[5]:
        result += (
            f"<link name='seg_f'><pose>{left_x:.4f} {(top_y+mid_y)/2:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_w:.4f} {v_len:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # G - middle horizontal
    if segs[6]:
        result += (
            f"<link name='seg_g'><pose>{cx:.4f} {mid_y:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_len:.4f} {seg_w:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    
    return result


def build_bw_target_sdf(model_name: str, size: float = 0.61, digit: int = 1) -> str:
    """Build SDF for black-white GCP target with X pattern and number.
    
    Creates a 24" (0.61m) square target with:
    - White base
    - Black triangles on top and bottom (created with strips)
    - White triangles on left and right  
    - A digit (0-9) with underline in bottom-right quadrant
    
    Args:
        model_name: Name for the model
        size: Side length in meters (default 0.61m = 24")
        digit: Number to display (0-9)
    """
    half = size / 2.0
    thickness = 0.01  # 1cm thick plate
    layer_z = thickness / 2 + 0.001  # Z position for pattern layers
    
    # Build triangles using horizontal strips
    # Each strip gets narrower as we approach the center
    num_strips = 10
    strip_height = half / num_strips
    
    # Generate strips for top triangle (black)
    top_strips = ""
    for i in range(num_strips):
        # y position from center toward top
        y_pos = (i + 0.5) * strip_height
        # Width decreases linearly from full width at top to 0 at center
        # At y=half (top edge), width = size
        # At y=0 (center), width = 0
        progress = 1.0 - (y_pos / half)  # 1 at center, 0 at top
        strip_width = size * (1.0 - progress * 0.95)  # Don't go to 0, leave small tip
        
        top_strips += (
            f"<link name='top_strip_{i}'>"
            f"<pose>0 {y_pos:.4f} {layer_z:.4f} 0 0 0</pose>"
            f"<visual name='visual'>"
            f"<geometry><box><size>{strip_width:.4f} {strip_height:.4f} 0.002</size></box></geometry>"
            f"<material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material>"
            f"</visual>"
            f"</link>"
        )
    
    # Generate strips for bottom triangle (black)
    bottom_strips = ""
    for i in range(num_strips):
        # y position from center toward bottom (negative y)
        y_pos = -((i + 0.5) * strip_height)
        progress = 1.0 - (abs(y_pos) / half)
        strip_width = size * (1.0 - progress * 0.95)
        
        bottom_strips += (
            f"<link name='bottom_strip_{i}'>"
            f"<pose>0 {y_pos:.4f} {layer_z:.4f} 0 0 0</pose>"
            f"<visual name='visual'>"
            f"<geometry><box><size>{strip_width:.4f} {strip_height:.4f} 0.002</size></box></geometry>"
            f"<material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material>"
            f"</visual>"
            f"</link>"
        )
    
    # Number in bottom-right white area (positioned in right triangle)
    num_x = half * 0.65  # More to the right
    num_y = -half * 0.05
    num_size = size * 0.42  # Bigger digit
    
    # Build digit using 7-segment style boxes
    number_elements = _build_digit_sdf(digit, num_x, num_y, num_size, layer_z + 0.001)
    
    # Add underline below the digit (fixed size, not tied to num_size)
    underline_y = num_y - num_size * 0.75
    underline_w = size * 0.18  # Fixed width
    underline_h = size * 0.025  # Fixed height
    number_elements += (
        f"<link name='underline'>"
        f"<pose>{num_x:.4f} {underline_y:.4f} {layer_z + 0.001:.4f} 0 0 0</pose>"
        f"<visual name='visual'>"
        f"<geometry><box><size>{underline_w:.4f} {underline_h:.4f} 0.002</size></box></geometry>"
        f"<material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>"
        f"</visual>"
        f"</link>"
    )
    
    return (
        f"<sdf version='1.7'>"
        f"<model name='{model_name}'>"
        f"<static>true</static>"
        
        # Base white plate
        f"<link name='base'>"
        f"<pose>0 0 0 0 0 0</pose>"
        f"<collision name='collision'>"
        f"<geometry><box><size>{size} {size} {thickness}</size></box></geometry>"
        f"</collision>"
        f"<visual name='visual'>"
        f"<geometry><box><size>{size} {size} {thickness}</size></box></geometry>"
        f"<material>"
        f"<ambient>0.95 0.95 0.95 1</ambient>"
        f"<diffuse>0.95 0.95 0.95 1</diffuse>"
        f"</material>"
        f"</visual>"
        f"</link>"
        
        # Top triangle strips
        f"{top_strips}"
        
        # Bottom triangle strips  
        f"{bottom_strips}"
        
        # Number and underline
        f"{number_elements}"
        
        f"</model>"
        f"</sdf>"
    )

def build_red_white_target_sdf(model_name: str, size: float = 5.0, num_rings: int = 8) -> str:
    """Build SDF for a red-white bullseye target for Package Drop/Delivery missions.

    Creates a circular target with alternating red and white concentric rings
    and a red center dot, matching competition spec (~5m diameter).

    The pattern from outside inward is:
        white base → red → white → red → white → red (center)

    Args:
        model_name: Name for the Gazebo model.
        size:       Diameter in meters (default 5.0m per competition rules).
        num_rings:  Number of visible ring layers over the white base (default 10,
                    giving 5 red + 5 white alternating bands + red center dot).
    """
    radius = size / 2.0
    thickness = 0.02        # 2 cm thick base plate
    layer_step = 0.002      # 2 mm z-separation per layer to prevent z-fighting

    RED   = "0.85 0.05 0.05 1"
    WHITE = "0.95 0.95 0.95 1"

    links = ""

    # --- White square tarp base (full size x size) ---
    base_z = thickness / 2.0
    links += (
        f"<link name='base'>"
        f"<pose>0 0 {base_z:.4f} 0 0 0</pose>"
        f"<collision name='collision'>"
        f"<geometry><box><size>{size:.4f} {size:.4f} {thickness:.4f}</size></box></geometry>"
        f"</collision>"
        f"<visual name='visual'>"
        f"<geometry><box><size>{size:.4f} {size:.4f} {thickness:.4f}</size></box></geometry>"
        f"<material><ambient>{WHITE}</ambient><diffuse>{WHITE}</diffuse></material>"
        f"</visual>"
        f"</link>"
    )

    # --- Concentric ring overlays: red outermost, alternating inward ---
    # Divide radius evenly among (num_rings + 1) bands so the innermost band
    # is also one unit wide (it becomes the center dot below).
    band_width = radius / (num_rings + 1)

    for i in range(num_rings):
        # Ring index 0 = outermost red ring, increases inward
        ring_radius = radius - i * band_width
        color = RED if (i % 2 == 0) else WHITE
        z = thickness + (i + 1) * layer_step
        links += (
            f"<link name='ring_{i}'>"
            f"<pose>0 0 {z:.4f} 0 0 0</pose>"
            f"<visual name='visual'>"
            f"<geometry><cylinder><radius>{ring_radius:.4f}</radius><length>0.001</length></cylinder></geometry>"
            f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>"
            f"</visual>"
            f"</link>"
        )

    # --- Center red dot ---
    center_radius = band_width
    center_z = thickness + (num_rings + 1) * layer_step
    links += (
        f"<link name='center'>"
        f"<pose>0 0 {center_z:.4f} 0 0 0</pose>"
        f"<visual name='visual'>"
        f"<geometry><cylinder><radius>{center_radius:.4f}</radius><length>0.001</length></cylinder></geometry>"
        f"<material><ambient>{RED}</ambient><diffuse>{RED}</diffuse></material>"
        f"</visual>"
        f"</link>"
    )

    return (
        f"<sdf version='1.7'>"
        f"<model name='{model_name}'>"
        f"<static>true</static>"
        f"{links}"
        f"</model>"
        f"</sdf>"
    )


def spawn_sdf(
    sdf_str: str,
    model_name: str,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    world_name: str = "map",
) -> bool:
    """Spawn an SDF model into a running Gazebo simulation.

    Uses the same ``gz service`` CLI approach as the TargetSpawner node.
    No ROS node or context is required; the function is safe to call from
    any thread or standalone script as long as ``gz`` is on PATH and
    Gazebo is running.

    Args:
        sdf_str:    Raw SDF XML string (e.g. returned by ``build_*_sdf`` helpers).
        model_name: Unique name to give the spawned entity.
        x:          World-frame X position in metres.
        y:          World-frame Y position in metres.
        z:          World-frame Z position in metres.
        world_name: Gazebo world name (default ``"default"``).

    Returns:
        ``True`` if Gazebo confirmed the spawn, ``False`` otherwise.
    """
    # Escape quotes for shell (matches target_spawner._spawn_via_gz_cli)
    sdf_escaped = sdf_str.replace('"', '\\"')

    req_msg = (
        f'sdf: "{sdf_escaped}" '
        f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}}} '
        f'name: "{model_name}" '
        f'allow_renaming: true'
    )

    cmd = [
        "gz", "service",
        "-s", f"/world/{world_name}/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000",
        "--req", req_msg
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
        if result.returncode == 0 and "true" in result.stdout.lower():
            _logger.info(f"Spawned '{model_name}' at ({x:.2f}, {y:.2f}, {z:.2f})")
            return True
        else:
            _logger.warning(f"Spawn failed: {result.stderr or result.stdout}")
            return False
    except subprocess.TimeoutExpired:
        _logger.warning("Spawn command timed out")
        return False
    except FileNotFoundError:
        _logger.error("'gz' command not found - is Gazebo installed?")
        return False

if __name__ == "__main__":
    # Example usage: spawn a target at the origin
    sdf = build_red_white_target_sdf("test_target", size=5.0)
    spawn_sdf(sdf, "test_target", x=5, y=5, z=1)