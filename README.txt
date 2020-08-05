回放ZED点云数据
rosbag play -l -r 1 rosbag/2020-0417水池，手推ROV前后移动/2020-04-02-11-35-39.bag

有用的topics
/zed/zed_node/point_cloud/cloud_registered
/zed/zed_node/left/image_rect_color

录包
rosbag record -O cloud /zed/zed_node/point_cloud/cloud_registered


rostopic pub /republish_cxx_node/path visualization_msgs/Marker "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'map'
ns: ''
id: 0
type: 4
action: 0
pose:
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
scale: {x: 0.002, y: 0.0, z: 0.0}
color: {r: 0.0, g: 1.0, b: 0.0, a: 1.0}
lifetime: {secs: 0, nsecs: 0}
frame_locked: false
points:
- {x: 0.1, y: 0.0, z: 0.0}
- {x: 0.2, y: 0.0, z: 0.0}
- {x: 0.1, y: 0.1, z: 0.0}
colors:
- {r: 1.0, g: 0.0, b: 0.0, a: 1.0}
- {r: 1.0, g: 0.0, b: 0.0, a: 1.0}
- {r: 1.0, g: 0.0, b: 0.0, a: 1.0}
text: ''
mesh_resource: ''
mesh_use_embedded_materials: false" 
