import math

# -----------------------------
# PARAMETERS
# -----------------------------
STRAIGHT_LEN = 14.0          # meters, length of each straight section
TURN_RADIUS = 6.0            # meters, centerline turn radius
WIDTH_MIN = 3.048            # 10 ft
WIDTH_MAX = 6.096            # 20 ft

SEG_LEN = 0.8                # length of each painted lane segment
SEG_WIDTH = 0.10             # painted line thickness
SEG_HEIGHT = 0.005           # painted line height
STEP = 0.8                   # spacing along the curve

Z = 0.001                    # slightly above ground to avoid z-fighting

# Arc centers
LEFT_CX = -STRAIGHT_LEN / 2.0
RIGHT_CX = STRAIGHT_LEN / 2.0

# Total centerline length
TOTAL_LEN = 2.0 * STRAIGHT_LEN + 2.0 * math.pi * TURN_RADIUS

def width_at_s(s):
    """
    Smoothly vary width around the course.
    Produces values between WIDTH_MIN and WIDTH_MAX.
    """
    t = s / TOTAL_LEN
    blend = 0.5 * (1.0 - math.cos(2.0 * math.pi * t))  # 0..1 smooth
    return WIDTH_MIN + (WIDTH_MAX - WIDTH_MIN) * blend

def centerline_pose(s):
    """
    Returns (x, y, yaw) for centerline position at arc length s.
    Track layout:
      top straight: left -> right
      right semicircle: top -> bottom
      bottom straight: right -> left
      left semicircle: bottom -> top
    """
    s1 = STRAIGHT_LEN
    s2 = s1 + math.pi * TURN_RADIUS
    s3 = s2 + STRAIGHT_LEN
    s4 = s3 + math.pi * TURN_RADIUS

    if s < s1:
        # top straight
        x = LEFT_CX + s
        y = TURN_RADIUS
        yaw = 0.0
        return x, y, yaw

    elif s < s2:
        # right semicircle
        u = (s - s1) / TURN_RADIUS  # 0..pi
        theta = math.pi / 2.0 - u   # +pi/2 down to -pi/2
        x = RIGHT_CX + TURN_RADIUS * math.cos(theta)
        y = TURN_RADIUS * math.sin(theta)
        yaw = theta - math.pi / 2.0
        return x, y, yaw

    elif s < s3:
        # bottom straight
        u = s - s2
        x = RIGHT_CX - u
        y = -TURN_RADIUS
        yaw = math.pi
        return x, y, yaw

    else:
        # left semicircle
        u = (s - s3) / TURN_RADIUS  # 0..pi
        theta = -math.pi / 2.0 - u  # -pi/2 down to -3pi/2
        x = LEFT_CX + TURN_RADIUS * math.cos(theta)
        y = TURN_RADIUS * math.sin(theta)
        yaw = theta - math.pi / 2.0
        return x, y, yaw

def offset_point(x, y, yaw, offset):
    """
    Offset point perpendicular to heading.
    Positive offset = left side of path
    Negative offset = right side of path
    """
    nx = -math.sin(yaw)
    ny =  math.cos(yaw)
    return x + offset * nx, y + offset * ny

def visual_block(name, x, y, yaw):
    return f'''        <visual name="{name}">
          <pose>{x:.4f} {y:.4f} {Z:.4f} 0 0 {yaw:.6f}</pose>
          <geometry>
            <box>
              <size>{SEG_LEN:.4f} {SEG_WIDTH:.4f} {SEG_HEIGHT:.4f}</size>
            </box>
          </geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>'''

def generate_lane_blocks():
    blocks = []
    s = 0.0
    i = 0
    while s < TOTAL_LEN:
        x, y, yaw = centerline_pose(s)
        width = width_at_s(s)

        outer_x, outer_y = offset_point(x, y, yaw,  width / 2.0)
        inner_x, inner_y = offset_point(x, y, yaw, -width / 2.0)

        blocks.append(visual_block(f"outer_lane_{i:03d}", outer_x, outer_y, yaw))
        blocks.append(visual_block(f"inner_lane_{i:03d}", inner_x, inner_y, yaw))

        s += STEP
        i += 1

    return "\n".join(blocks)

if __name__ == "__main__":
    print(generate_lane_blocks())