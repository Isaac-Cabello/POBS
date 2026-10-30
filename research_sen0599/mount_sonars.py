from pathlib import Path
import math
import sys

sys.path.insert(0, str(Path('ros2_ws/src/pobs_sonar')))
from pobs_sonar.mounting import SONAR_MOUNTS

model_path = Path('ros2_ws/src/pobs_gazebo/models/pobs/model.sdf')
text = model_path.read_text()
marker = '    </link>\n    <plugin filename="gz-sim-hydrodynamics-system"'
assert text.count(marker) == 1
assert '<sensor name="sonar_left"' not in text
parts = ['      <!-- Provisional front sonar mounts: +X forward, +Y left, +Z up. -->']
for name, (x, y, z, roll, pitch, yaw) in SONAR_MOUNTS.items():
    bx = x - 0.017 * math.cos(pitch)
    bz = z + 0.017 * math.sin(pitch)
    for suffix, cx, cz, length, radius, color in (
        ('housing', bx, bz, 0.034, 0.014, '0.08 0.10 0.12 1'),
        ('face', x, z, 0.002, 0.013, '0.80 0.85 0.88 1'),
    ):
        parts.append(f'''      <visual name="sonar_{name}_{suffix}">
        <pose>{cx:.9f} {y} {cz:.9f} 0 {pitch + math.pi / 2:.16f} 0</pose>
        <geometry>
          <cylinder><radius>{radius}</radius><length>{length}</length></cylinder>
        </geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>''')
    if name == 'depth':
        continue
    parts.append(f'''      <visual name="sonar_{name}_bracket">
        <pose>0.404 {y} 0 0 0 0</pose>
        <geometry><box><size>0.012 0.04 0.05</size></box></geometry>
        <material>
          <ambient>0.25 0.28 0.30 1</ambient>
          <diffuse>0.25 0.28 0.30 1</diffuse>
        </material>
      </visual>
      <sensor name="sonar_{name}" type="gpu_lidar">
        <pose>{x} {y} {z} {roll} {pitch:.16f} {yaw}</pose>
        <topic>/pobs/sonar/sonar_{name}/scan</topic>
        <update_rate>10</update_rate>
        <always_on>true</always_on>
        <visualize>true</visualize>
        <lidar>
          <scan>
            <horizontal>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </horizontal>
          </scan>
          <range><min>0.05</min><max>6.0</max><resolution>0.001</resolution></range>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>
        </lidar>
      </sensor>''')
text = text.replace(marker, '\n'.join(parts) + '\n' + marker)
model_path.write_text(text)
