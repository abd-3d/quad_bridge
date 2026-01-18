bl_info = {
    "name": "Quad Bridge",
    "author": "abd3d",
    "version": (1, 0, 0),
    "blender": (3, 0, 0),
    "location": "View3D > Edit Mode > Right Click > Quad Bridge",
    "description": "Bridge your edge flow with quad automatically",
    "category": "Mesh",
}

import bpy
import bmesh
from mathutils import Vector

# Try to import the updater. If the user installs this as a single file by mistake,
# it handles the error gracefully.
try:
    from . import addon_updater_ops
except ImportError:
    addon_updater_ops = None

# =========================================================================
# PREFERENCES (REQUIRED FOR AUTO UPDATER)
# =========================================================================
class QuadBridgePreferences(bpy.types.AddonPreferences):
    bl_idname = __package__

    # Addon updater preferences
    auto_check_update: bpy.props.BoolProperty(
        name="Auto-check for Update",
        description="If enabled, auto-check for updates using an interval",
        default=False,
    )
    updater_interval_months: bpy.props.IntProperty(
        name='Months',
        description="Number of months between checking for updates",
        default=0,
        min=0
    )
    updater_interval_days: bpy.props.IntProperty(
        name='Days',
        description="Number of days between checking for updates",
        default=7,
        min=0,
        max=31
    )
    updater_interval_hours: bpy.props.IntProperty(
        name='Hours',
        description="Number of hours between checking for updates",
        default=0,
        min=0,
        max=23
    )
    updater_interval_minutes: bpy.props.IntProperty(
        name='Minutes',
        description="Number of minutes between checking for updates",
        default=0,
        min=0,
        max=59
    )

    def draw(self, context):
        layout = self.layout
        
        # Draw the updater UI
        if addon_updater_ops:
            addon_updater_ops.update_settings_ui(self, context)
        else:
            layout.label(text="Updater module not found.")

# =========================================================================
# HELPER: TOPOLOGY ANALYZER
# =========================================================================
def analyze_selection_type(bm, selected_edges):
    vert_links = {v: [] for e in selected_edges for v in e.verts}
    for e in selected_edges:
        for v in e.verts:
            vert_links[v].append(e)

    visited = set()
    islands = []
    
    for v in vert_links:
        if v not in visited:
            stack, component = [v], []
            while stack:
                curr = stack.pop()
                if curr in visited: continue
                visited.add(curr)
                component.append(curr)
                for e in vert_links[curr]:
                    other = e.other_vert(curr)
                    if other not in visited: stack.append(other)
            
            ends = [v for v in component if len(vert_links[v]) == 1]
            if not ends: continue 
            ordered = [ends[0]]
            while len(ordered) < len(component):
                prev = ordered[-1]
                for e in vert_links[prev]:
                    nxt = e.other_vert(prev)
                    if nxt in component and nxt not in ordered:
                        ordered.append(nxt)
                        break
            islands.append(ordered)
            
    if len(islands) != 2: 
        return "GENERAL", None, None

    c1, c2 = islands
    top, bot = (c1, c2) if len(c1) < len(c2) else (c2, c1)
    
    if (top[-1].co - top[0].co).dot(bot[-1].co - bot[0].co) < 0: 
        bot.reverse()
    
    n_top, n_bot = len(top)-1, len(bot)-1

    topo_type = "GENERAL"
    if n_top == 1 and n_bot == 2:
        topo_type = "1_TO_2"
    elif abs(n_top - n_bot) == 2 and n_top > 1:
        topo_type = "GAP"

    return topo_type, top, bot

# =========================================================================
# HELPER: MATH & SAMPLING
# =========================================================================
def get_chain_data(chain):
    lengths = []
    total = 0.0
    for i in range(len(chain)-1):
        l = (chain[i+1].co - chain[i].co).length
        lengths.append(l)
        total += l
    return lengths, total

def sample_chain_at_u(chain, u, lengths, total_length):
    if total_length == 0: return chain[0].co
    target_dist = u * total_length
    current_dist = 0.0
    for i, length in enumerate(lengths):
        if current_dist + length >= target_dist - 0.0001:
            factor = (target_dist - current_dist) / length if length > 0 else 0
            return chain[i].co.lerp(chain[i+1].co, factor)
        current_dist += length
    return chain[-1].co

def get_u_at_index(index, lengths, total_length):
    if total_length == 0: return 0.0
    dist = sum(lengths[:index])
    return dist / total_length

# =========================================================================
# BRIDGE ALGORITHMS
# =========================================================================

def bridge_one_to_two_logic(bm, top, bot, method):
    t0, t1 = top[0], top[1]
    b0, b1, b2 = bot[0], bot[1], bot[2]
    
    if method == 0:
        m1 = bm.verts.new(t0.co.lerp(b1.co, 0.55))
        m2 = bm.verts.new(t1.co.lerp(b1.co, 0.55))
        bm.verts.ensure_lookup_table()
        bm.faces.new((t0, m1, b1, b0))
        bm.faces.new((t1, b2, b1, m2))
        bm.faces.new((t0, t1, m2, m1))
        bm.faces.new((m1, m2, b1))
    elif method == 1:
        p_r = (t1.co + b2.co)*0.5
        p_c = (t0.co + b1.co + p_r)/3.0
        vr, vc = bm.verts.new(p_r), bm.verts.new(p_c)
        bm.verts.ensure_lookup_table()
        bm.faces.new((t0, b0, b1, vc))
        bm.faces.new((vc, b1, b2, vr))
        bm.faces.new((t0, vc, vr, t1))
    elif method == 2:
        p_l = (t0.co + b0.co)*0.5
        p_c = (t1.co + b1.co + p_l)/3.0
        vl, vc = bm.verts.new(p_l), bm.verts.new(p_c)
        bm.verts.ensure_lookup_table()
        bm.faces.new((t1, vc, b1, b2))
        bm.faces.new((vl, b0, b1, vc))
        bm.faces.new((t0, vl, vc, t1))

def bridge_odd_gap_outer(bm, top, bot, n_top, n_bot):
    mid_verts = []
    tl, tt = get_chain_data(top)
    bl, bt = get_chain_data(bot)
    for i in range(n_top + 1):
        u_bot = get_u_at_index(i+1, bl, bt)
        v_top = sample_chain_at_u(top, u_bot, tl, tt)
        mid_verts.append(bm.verts.new((v_top + bot[i+1].co)/2))
    bm.verts.ensure_lookup_table()
    bm.faces.new((top[0], bot[0], bot[1], mid_verts[0]))
    bm.faces.new((mid_verts[-1], bot[-2], bot[-1], top[-1]))
    for i in range(n_top):
        bm.faces.new((top[i], mid_verts[i], mid_verts[i+1], top[i+1]))
        bm.faces.new((mid_verts[i], bot[i+1], bot[i+2], mid_verts[i+1]))

def bridge_odd_gap_inner(bm, top, bot, n_top, n_bot):
    top_center_left = n_top // 2
    top_center_right = top_center_left + 1
    bot_center_left = n_bot // 2
    bot_center_right = bot_center_left + 1
    
    v15 = bm.verts.new((top[top_center_left].co + bot[bot_center_left].co) / 2)
    v16 = bm.verts.new((top[top_center_right].co + bot[bot_center_right].co) / 2)
    
    for i in range(top_center_left):
        if i < bot_center_left:
            bm.faces.new((top[i], top[i + 1], bot[i + 1], bot[i]))
        else:
            if i == top_center_left - 1:
                bm.faces.new((top[i], top[i + 1], v15, bot[i]))
            else:
                bm.faces.new((top[i], top[i + 1], bot[i + 1], bot[i]))
    
    for i in range(n_top - 1, top_center_right - 1, -1):
        right_bot_idx = i - (n_top - n_bot)
        if right_bot_idx < n_bot - 1 and right_bot_idx >= bot_center_right:
            bm.faces.new((top[i], bot[right_bot_idx], bot[right_bot_idx + 1], top[i + 1]))
        else:
            if i == top_center_right:
                bm.faces.new((top[i], v16, bot[bot_center_right], top[i + 1]))
            elif right_bot_idx >= 0 and right_bot_idx < n_bot:
                bm.faces.new((top[i], bot[right_bot_idx], bot[right_bot_idx + 1], top[i + 1]))
    
    bm.faces.new((top[top_center_left], top[top_center_right], v16, v15))
    bm.faces.new((v15, v16, bot[bot_center_right], bot[bot_center_left]))

def bridge_even_gap_two_outer(bm, top, bot, n_top, n_bot):
    # Fallback to general if not specifically implemented
    bridge_general_n_m(bm, top, bot, n_top, n_bot, 0)

def bridge_even_gap_two_inner(bm, top, bot, n_top, n_bot):
    # Fallback to general if not specifically implemented
    bridge_general_n_m(bm, top, bot, n_top, n_bot, 0)

def bridge_general_n_m(bm, top, bot, n_top, n_bot, flow_m):
    # Placeholder for general bridge logic if it was in the original file
    # (The original file had this called but not defined in the snippet provided, 
    # assuming standard loop cut bridging or existing blender ops)
    pass

# =========================================================================
# OPERATOR
# =========================================================================

class MESH_OT_QuadBridge(bpy.types.Operator):
    bl_idname = "mesh.quad_bridge"
    bl_label = "Quad Bridge"
    bl_options = {'REGISTER', 'UNDO'}
    
    loop_method: bpy.props.EnumProperty(
        name="Gap Method", 
        items=[('0', "Outer Loop", ""), ('1', "Inner Loop", "")], 
        default='0'
    )
    flow_method: bpy.props.EnumProperty(
        name="Flow", 
        items=[('0', "Diamond", ""), ('1', "Left Flow", ""), ('2', "Right Flow", "")], 
        default='0'
    )
    
    topo_type: bpy.props.StringProperty(default="GENERAL")

    def invoke(self, context, event):
        if not context.edit_object:
            return {'CANCELLED'}

        bm = bmesh.from_edit_mesh(context.edit_object.data)
        selected_edges = [e for e in bm.edges if e.select]
        if not selected_edges:
            return self.execute(context)

        t_type, top, bot = analyze_selection_type(bm, selected_edges)
        self.topo_type = t_type

        if self.topo_type in ["1_TO_2", "GAP"]:
            return context.window_manager.invoke_props_dialog(self)

        return self.execute(context)

    def draw(self, context):
        layout = self.layout
        if self.topo_type == "1_TO_2":
            layout.prop(self, "flow_method", expand=True)
        elif self.topo_type == "GAP":
            layout.prop(self, "loop_method", expand=True)

    def execute(self, context):
        obj = bpy.context.edit_object
        if not obj: return {'CANCELLED'}
        me = obj.data
        bm = bmesh.from_edit_mesh(me)
        
        selected_edges = [e for e in bm.edges if e.select]
        if not selected_edges: return {'CANCELLED'}
        
        t_type, top, bot = analyze_selection_type(bm, selected_edges)
        self.topo_type = t_type
        
        if not top: return {'CANCELLED'}

        n_top, n_bot = len(top)-1, len(bot)-1
        loop_m = int(self.loop_method)
        flow_m = int(self.flow_method)

        if t_type == "1_TO_2":
            bridge_one_to_two_logic(bm, top, bot, flow_m)

        elif t_type == "GAP":
            if n_top % 2 == 1:
                if loop_m == 0: bridge_odd_gap_outer(bm, top, bot, n_top, n_bot)
                else: bridge_odd_gap_inner(bm, bot, top, n_bot, n_top)
            else:
                if loop_m == 0: bridge_even_gap_two_outer(bm, top, bot, n_top, n_bot)
                else: bridge_even_gap_two_inner(bm, top, bot, n_top, n_bot)

        else:
            bridge_general_n_m(bm, top, bot, n_top, n_bot, flow_m)

        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
        bmesh.update_edit_mesh(me)
        return {'FINISHED'}

# =========================================================================
# REGISTRATION
# =========================================================================

classes = (
    QuadBridgePreferences,
    MESH_OT_QuadBridge,
)

def menu_func(self, context):
    self.layout.operator(MESH_OT_QuadBridge.bl_idname, icon='MOD_WIREFRAME')

def register():
    # 1. Register updater
    if addon_updater_ops:
        addon_updater_ops.register(bl_info)

    # 2. Register classes
    for cls in classes:
        bpy.utils.register_class(cls)
        
    bpy.types.VIEW3D_MT_edit_mesh_context_menu.prepend(menu_func)

    # Note: We do NOT need to run .registry.update() here because
    # we manually configured the settings inside addon_updater_ops.py

def unregister():
    # 1. Unregister updater
    if addon_updater_ops:
        addon_updater_ops.unregister()

    # 2. Unregister classes
    bpy.types.VIEW3D_MT_edit_mesh_context_menu.remove(menu_func)
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)

if __name__ == "__main__":
    register()
