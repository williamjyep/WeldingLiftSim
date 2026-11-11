import vpython as vp
import math

# --- SIMULATION CONSTANTS & DIMENSIONS ---
CHASSIS_W = 0.600  # 600mm width
CHASSIS_L = 0.770  # 770mm length
CHASSIS_T = 0.050  # 50mm thickness
LEG_LENGTH = 0.600 # 600mm leg height
AXIS_OFFSET = 0.040 # 40mm offset from ends
GEAR_RATIO = 4.0   # 1:4 gear ratio

# Masses and Forces
MASS_SPOOL_ROBOT = 85.0  # 65kg spool + 20kg robot chassis
PULL_FORCE_KG = 35.0
G = 9.81

F_DOWN = MASS_SPOOL_ROBOT * G
F_PULL_Z = PULL_FORCE_KG * G  # Force purely in the bending direction

# --- VPYTHON 3D SCENE SETUP ---
vp.scene.title = "<b>Ship Welding Robot - Folding Dynamics</b>\nRight-click and drag to orbit. Scroll to zoom."
vp.scene.width = 900
vp.scene.height = 600
vp.scene.background = vp.color.gray(0.2)
vp.scene.forward = vp.vec(-1, -0.5, -1)

# --- 3D OBJECTS ---
# Chassis
chassis = vp.box(pos=vp.vec(0, LEG_LENGTH, 0), 
                 size=vp.vec(CHASSIS_W, CHASSIS_T, CHASSIS_L), 
                 color=vp.color.cyan, opacity=0.8)

# Axles connecting the pairs
z_axle_1 = (CHASSIS_L / 2) - AXIS_OFFSET
z_axle_2 = -(CHASSIS_L / 2) + AXIS_OFFSET

axle1 = vp.cylinder(pos=vp.vec(-CHASSIS_W/2, LEG_LENGTH, z_axle_1), axis=vp.vec(CHASSIS_W, 0, 0), radius=0.025, color=vp.color.blue)
axle2 = vp.cylinder(pos=vp.vec(-CHASSIS_W/2, LEG_LENGTH, z_axle_2), axis=vp.vec(CHASSIS_W, 0, 0), radius=0.025, color=vp.color.blue)

# Hardstops (Initially hidden)
hardstop_thickness = 0.03
hardstop_drop = 0.04
hardstop_offset = 0.04  # Distance outward from the axle
hardstop_front = vp.box(pos=vp.vec(0, LEG_LENGTH - hardstop_drop, z_axle_1 + hardstop_offset), 
                        size=vp.vec(CHASSIS_W, hardstop_thickness, hardstop_thickness), 
                        color=vp.color.green, visible=False)
hardstop_back = vp.box(pos=vp.vec(0, LEG_LENGTH - hardstop_drop, z_axle_2 - hardstop_offset), 
                       size=vp.vec(CHASSIS_W, hardstop_thickness, hardstop_thickness), 
                       color=vp.color.green, visible=False)

# Legs and Feet
leg_radius = 0.03
legs = []
feet = []

# Pair 1 (Front)
for x_pos in [CHASSIS_W/2 - AXIS_OFFSET, -CHASSIS_W/2 + AXIS_OFFSET]:
    leg = vp.cylinder(pos=vp.vec(x_pos, LEG_LENGTH, z_axle_1), axis=vp.vec(0, -LEG_LENGTH, 0), radius=leg_radius, color=vp.color.gray(0.7))
    foot = vp.box(pos=leg.pos + leg.axis, size=vp.vec(0.06, 0.04, 0.06), color=vp.color.black)
    legs.append((leg, 1)) # Tag with pair 1
    feet.append(foot)

# Pair 2 (Back)
for x_pos in [CHASSIS_W/2 - AXIS_OFFSET, -CHASSIS_W/2 + AXIS_OFFSET]:
    leg = vp.cylinder(pos=vp.vec(x_pos, LEG_LENGTH, z_axle_2), axis=vp.vec(0, -LEG_LENGTH, 0), radius=leg_radius, color=vp.color.gray(0.7))
    foot = vp.box(pos=leg.pos + leg.axis, size=vp.vec(0.06, 0.04, 0.06), color=vp.color.black)
    legs.append((leg, 2)) # Tag with pair 2
    feet.append(foot)

# --- FORCE VECTORS ---
arrow_scale = 0.0005
arrow_down = vp.arrow(pos=chassis.pos, axis=vp.vec(0, -F_DOWN * arrow_scale, 0), color=vp.color.red, shaftwidth=0.015)
arrow_pz = vp.arrow(pos=chassis.pos, axis=vp.vec(0, 0, F_PULL_Z * arrow_scale), color=vp.color.yellow, shaftwidth=0.015)

# --- UI & STATE VARIABLES ---
status_text = vp.label(pos=vp.vec(0, LEG_LENGTH + 0.3, 0), text="Calculating...", box=True, height=16)

target_theta = 0.0
current_theta = 0.0

def update_ui(evt=None):
    global target_theta
    motor_torque = torque_slider.value
    hardstop_enabled = hardstop_check.checked
    
    # Toggle hardstop visibility
    hardstop_front.visible = hardstop_enabled
    hardstop_back.visible = hardstop_enabled
    
    # Effective torque: Motor * Gear Ratio * 2 Motors (one per axle)
    effective_holding_torque = motor_torque * GEAR_RATIO * 2 
    
    # Moment = Force * Lever Arm (Leg Length)
    folding_moment = F_PULL_Z * LEG_LENGTH
    
    if hardstop_enabled:
        # One pair will be pushed into the hardstop, locking the chassis
        hold_status = "MECHANICAL HARDSTOP ENGAGED"
        status_text.color = vp.color.green
        target_theta = 0.0
    elif effective_holding_torque >= folding_moment:
        hold_status = "MOTORS HOLDING"
        status_text.color = vp.color.green
        target_theta = 0.0
    else:
        hold_status = "MOTORS YIELDING (Folding!)"
        status_text.color = vp.color.red
        target_theta = 1.3  # Fold angle in radians (~75 degrees)

    status_text.text = (
        f"Input Motor Torque (per motor): {motor_torque:.1f} Nm\n"
        f"Total Axis Torque (both axles, 1:4 gear): {effective_holding_torque:.1f} Nm\n"
        f"Folding Moment from 35kg Pull: {folding_moment:.1f} Nm\n"
        f"Status: {hold_status}"
    )
    torque_readout.text = f" Motor Torque: {motor_torque:.1f} Nm"

vp.scene.append_to_caption('\n<b>Controls:</b>\n')
hardstop_check = vp.checkbox(bind=update_ui, text='Enable Outer Hardstops (Prevents outward rotation)')
vp.scene.append_to_caption('\n\n')
torque_slider = vp.slider(min=0, max=100, value=25, length=400, bind=update_ui)
torque_readout = vp.wtext(text=f" Motor Torque: {torque_slider.value:.1f} Nm")

update_ui()

# --- ANIMATION LOOP ---
while True:
    vp.rate(60)
    
    # Smoothly animate the folding angle toward the target state
    current_theta += (target_theta - current_theta) * 0.05
    
    # Calculate new chassis height based on fold angle
    current_height = LEG_LENGTH * math.cos(current_theta)
    chassis.pos.y = current_height
    
    # Update Axles & Hardstops
    axle1.pos.y = current_height
    axle2.pos.y = current_height
    hardstop_front.pos.y = current_height - hardstop_drop
    hardstop_back.pos.y = current_height - hardstop_drop
    
    # Update Legs and Feet
    for i, (leg, pair) in enumerate(legs):
        leg.pos.y = current_height
        # Rotate axis to simulate folding backwards along the Z-axis
        leg.axis = vp.vec(0, -LEG_LENGTH * math.cos(current_theta), -LEG_LENGTH * math.sin(current_theta))
        feet[i].pos = leg.pos + leg.axis
        
    # Update Force Arrows to follow the chassis
    arrow_down.pos = chassis.pos
    arrow_pz.pos = chassis.pos
    status_text.pos = vp.vec(0, current_height + 0.3, 0)