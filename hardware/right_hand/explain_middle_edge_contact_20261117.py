"""Render and dimension the measured MODEL contact; no physical device access."""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
from pathlib import Path
import hashlib, json, sys
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
import numpy as np
import mujoco
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties, fontManager
from matplotlib.patches import Rectangle, ConnectionPatch
from research_rotation_body_hold_20261104 import restore
from audit_opposed_working_grip_20261117 import MODEL
from render_canonical_grip_evidence_20261105 import display_physical_cube, cube_display, png, extra

BASE = ROOT/'right_hand_rotation_20261117/opposed_lower_grip'
OUT = BASE/'middle_contact_explanation'
OUT.mkdir(exist_ok=True)
SOURCE = BASE/'core_support_chain_10s/audit_end_state.npz'
INSPECTION = BASE/'regrasp_state_inspection_extra.json'
reference = json.loads(INSPECTION.read_bytes())['core_support_chain_10s']
m = mujoco.MjModel.from_binary_path('model.mjb', assets={'model.mjb': MODEL.read_bytes()})
d = restore(m, dict(np.load(SOURCE, allow_pickle=False)))
cube, tip = m.geom('cube').id, m.geom('mf_tip').id
C = d.geom_xpos[cube].copy()
B = d.geom_xmat[cube].reshape(3, 3).copy()
contacts = []
for ci, contact in enumerate(d.contact):
    if {int(contact.geom1), int(contact.geom2)} == {cube, tip}:
        f = np.zeros(6)
        mujoco.mj_contactForce(m, d, ci, f)
        contacts.append((float(f[0]), np.array(contact.pos)))
force, point = max(contacts, key=lambda item: item[0])
local = (point-C)@B*1000
saved = next(c['cube_local_mm'] for c in reference['contacts'] if c['name']=='mf_tip')
if np.max(abs(local-saved))>.01:
    raise RuntimeError('Restored contact does not match the cited snapshot')
half = float(m.geom_size[cube][0]*1000)
margin = half-local[0]
if not 0 < margin < 1 or abs(local[2]+half)>.01:
    raise RuntimeError('Expected contact close to the positive-X edge on negative-Z face')

# Geometry and positions stay unchanged. Colors are display-only.
m.vis.global_.offwidth = 1200
m.vis.global_.offheight = 1000
display_physical_cube(m)
for g in range(m.ngeom):
    name = m.body(int(m.geom_bodyid[g])).name
    if name.startswith('mf_'):
        m.geom_matid[g] = -1
        m.geom_rgba[g] = [.10, .70, .42, .72]
    elif name.startswith('th_'):
        m.geom_matid[g] = -1
        m.geom_rgba[g] = [.77, .32, .30, .85]
    elif name.startswith('if_'):
        m.geom_matid[g] = -1
        m.geom_rgba[g] = [.31, .44, .65, .80]
m.geom_rgba[tip] = [.10, .80, .42, .35]
m.geom_matid[cube] = -1
m.geom_rgba[cube] = [.83, .87, .89, 1]
camera = mujoco.MjvCamera()
camera.lookat[:] = C-B[:, 1]*.002
camera.distance = .205
view = -B[:, 2]-.6*B[:, 1]+.2*B[:, 0]
view /= np.linalg.norm(view)
camera.azimuth = np.degrees(np.arctan2(view[1], view[0]))+180
camera.elevation = -np.degrees(np.arcsin(view[2]))
option = mujoco.MjvOption()
option.geomgroup[:] = [1, 1, 1, 0, 0, 0]
renderer = mujoco.Renderer(m, height=900, width=1100)
renderer.update_scene(d, camera=camera, scene_option=option)
print(json.dumps(dict(requested_view=view.tolist(), camera_pos=np.asarray(renderer.scene.camera[0].pos).tolist(),
                      target=C.tolist(), camera_azimuth=float(camera.azimuth), camera_elevation=float(camera.elevation))), flush=True)
cube_display(renderer.scene, C, B, False)
# Mark the entire side edge being measured, and the representative contact.
edge_a = C+B@np.array([half/1000, -half/1000, -half/1000])
edge_b = C+B@np.array([half/1000, half/1000, -half/1000])
direction = (edge_b-edge_a)/np.linalg.norm(edge_b-edge_a)
axis1 = B[:, 0]
axis2 = np.cross(direction, axis1)
extra(renderer.scene, mujoco.mjtGeom.mjGEOM_CAPSULE, [.00048, .0275, 0],
      (edge_a+edge_b)/2, np.column_stack([axis1, axis2, direction]), [1., .65, .04, 1.])
extra(renderer.scene, mujoco.mjtGeom.mjGEOM_SPHERE, [.00105, 0, 0],
      point-B[:, 2]*.0005, np.eye(3), [.86, .10, .42, 1.])
model_image = renderer.render().copy()
eyes = renderer.scene.camera
eye = np.mean([np.asarray(c.pos) for c in eyes], axis=0)
forward = np.mean([np.asarray(c.forward) for c in eyes], axis=0)
forward /= np.linalg.norm(forward)
up = np.mean([np.asarray(c.up) for c in eyes], axis=0)
up /= np.linalg.norm(up)
right_axis = np.cross(forward, up)
near = float(eyes[0].frustum_near)
top = float(eyes[0].frustum_top)
bottom = float(eyes[0].frustum_bottom)
relative = point-eye
depth = float(relative@forward)
focal = model_image.shape[0]*near/(top-bottom)
projected = np.array([model_image.shape[1]/2+focal*(relative@right_axis)/depth,
    model_image.shape[0]*top/(top-bottom)-focal*(relative@up)/depth])
if not (0 < projected[0] < model_image.shape[1] and 0 < projected[1] < model_image.shape[0]):
    raise RuntimeError('Contact outside explanation camera')
png(OUT/'model_contact_view.png', model_image)
renderer.close()

font_path = '/mnt/c/Windows/Fonts/malgun.ttf'
fontManager.addfont(font_path)
plt.rcParams.update({'font.family': FontProperties(fname=font_path).get_name(),
    'axes.unicode_minus': False, 'font.size': 12, 'savefig.facecolor': '#f7f9fc'})
fig = plt.figure(figsize=(18, 10), dpi=140, facecolor='#f7f9fc')
grid = fig.add_gridspec(2, 3, width_ratios=[1.25, 1., .92], height_ratios=[1., .12],
                       left=.025, right=.98, top=.81, bottom=.07, wspace=.22, hspace=.12)
ax_model = fig.add_subplot(grid[0, 0])
ax_face = fig.add_subplot(grid[0, 1])
ax_zoom = fig.add_subplot(grid[0, 2])
pink, orange, green = '#d92772', '#e59a16', '#248b5d'
fig.text(.035, .94, '중지 접촉점은 옆 모서리 바로 안쪽에 있다', fontsize=25, weight='bold', color='#142d42')
fig.text(.035, .885, f'55mm 큐브 · 대표 접촉점에서 옆 모서리까지 면을 따라 {margin:.3f}mm (약 0.35mm)', fontsize=17, color='#38576d')
fig.text(.035, .844, '0.35mm는 접촉점의 위치를 나타낸다. 아래 확대도에서 분홍 점과 노란 세로선 사이를 보면 된다.', fontsize=13, color='#38576d')
ax_model.imshow(model_image)
ax_model.scatter([projected[0]], [projected[1]], s=85, color=pink, edgecolor='white', linewidth=1.4, zorder=6)
ax_model.annotate('중지 접촉점\n(중지에 가려진 위치 표시)', projected,
    (projected[0]-290, projected[1]-150), fontsize=11, color=pink,
    bbox={'boxstyle':'round,pad=.35', 'facecolor':'white', 'alpha':.94, 'edgecolor':'none'},
    arrowprops={'arrowstyle':'->', 'color':pink, 'lw':1.6})
ax_model.axis('off')
ax_model.set_title('① 저장 모델의 실제 기하\n초록=중지 / 분홍=접촉점 / 노랑=해당 옆 모서리', fontsize=14, pad=13)

px, py = float(local[0]), float(-local[1])
ax_face.add_patch(Rectangle((-half, -half), half*2, half*2, facecolor='#e7eef3', edgecolor='#425e73', linewidth=2))
for p in (-half/3, half/3):
    ax_face.plot([p, p], [-half, half], color='#adbdc9', lw=1)
    ax_face.plot([-half, half], [p, p], color='#adbdc9', lw=1)
ax_face.plot([half, half], [-half, half], color=orange, lw=4, zorder=4)
ax_face.scatter([0], [0], s=50, facecolor='white', edgecolor='#536d7e', zorder=4)
ax_face.annotate('면 가운데', (0, 0), (-23, 10), arrowprops={'arrowstyle':'-', 'color':'#536d7e'}, color='#536d7e', fontsize=12)
ax_face.scatter([px], [py], s=90, facecolor=pink, edgecolor='white', linewidth=1.2, zorder=6)
ax_face.annotate('중지 접촉점', (px, py), (-21, -18), color=pink, fontsize=14,
                 arrowprops={'arrowstyle':'->', 'color':pink, 'lw':1.7})
ax_face.annotate('55mm', (-half, half+3.8), (0, half+3.8), ha='center', va='center', fontsize=12)
ax_face.annotate('', (-half, half+2.5), (half, half+2.5), arrowprops={'arrowstyle':'<->', 'color':'#536d7e'})
ax_face.text(0, -half-6, '중간층 아래쪽 · 옆 모서리 가까이', ha='center', color='#38576d', fontsize=12)
ax_face.set(xlim=(-33, 34), ylim=(-36, 36), aspect='equal')
ax_face.axis('off')
ax_face.set_title('② 중지가 닿는 면을 정면으로 펼친 좌표도\n위=윗면 쪽 / 격자=3×3 층 경계', fontsize=14, pad=13)

# The inset uses the same real model coordinates; only the scale is enlarged.
left, right = px-.75, half+.62
bottom, top = py-.68, py+.83
ax_zoom.add_patch(Rectangle((left, bottom), half-left, top-bottom,
    facecolor='#e7eef3', edgecolor='none'))
ax_zoom.axvline(half, color=orange, lw=4)
ax_zoom.scatter([px], [py], s=140, color=pink, edgecolor='white', linewidth=1.5, zorder=5)
dim_y = py+.34
ax_zoom.plot([px, px], [py+.04, dim_y+.06], color=pink, lw=1.2)
ax_zoom.annotate('', (px, dim_y), (half, dim_y),
    arrowprops={'arrowstyle':'<->', 'color':'#223e52', 'lw':1.6})
ax_zoom.text((px+half)/2, dim_y+.09, f'{margin:.2f} mm', ha='center', fontsize=19, weight='bold', color='#173347')
ax_zoom.annotate('대표 접촉점', (px, py), (left+.05, py-.36), fontsize=13, color=pink,
    arrowprops={'arrowstyle':'->', 'color':pink, 'lw':1.4})
ax_zoom.text(left+.04, top-.08, '면 안쪽', color='#38576d', fontsize=13, va='top')
ax_zoom.text(half+.11, top-.08, '면\n바깥쪽', color='#38576d', fontsize=12, va='top')
ax_zoom.text(half+.06, bottom+.10, '옆 모서리', rotation=90, fontsize=13, color='#a56500', va='bottom')
ax_zoom.set(xlim=(left, right), ylim=(bottom, top), aspect='equal')
ax_zoom.axis('off')
ax_zoom.set_title('③ 같은 접촉점의 확대도\n분홍 점이 노란 선 바로 안쪽에 있음', fontsize=14, pad=13)
link = ConnectionPatch(xyA=(px, py), coordsA=ax_face.transData,
    xyB=(left, py), coordsB=ax_zoom.transData, color=pink, lw=1.2, linestyle='--', alpha=.6)
fig.add_artist(link)
foot = fig.add_subplot(grid[1, :])
foot.axis('off')
foot.text(.01, .85, '분홍 점은 모델이 계산한 대표 접촉 위치다. 손끝 전체의 접촉 면적을 표시한 것은 아니다.', fontsize=13, color='#344f63')
foot.text(.01, .32, '출처: 작은 회전 뒤 10.05초 유지의 마지막 저장 상태 · 0.35mm를 인용한 바로 그 순간 · 실물 사진의 측정값이 아님', fontsize=12, color='#5a7080')
fig.savefig(OUT/'middle_contact_edge_explained.png', dpi=140)
fig.savefig(OUT/'middle_contact_edge_explained.svg')
plt.close(fig)
manifest = dict(status='source_bound_model_contact_explanation',
    source_path=SOURCE.relative_to(ROOT).as_posix(), source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
    model_sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest(),
    inspection_sha256=hashlib.sha256(INSPECTION.read_bytes()).hexdigest(),
    contact_geom='mf_tip', cube_contact_local_mm=local.tolist(), representative_normal_force_N=force,
    edge_local_x_mm=half, in_face_edge_margin_mm=margin, face_local_z_mm=-half,
    source_is_10second_snapshot_not_20second_snapshot=True,
    geometric_point_not_contact_patch=True, display_marker_radius_not_physical_measurement=True,
    image='middle_contact_edge_explained.png', devices_opened=False)
(OUT/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
print(json.dumps(manifest), flush=True)
