from ffmpeg_bindings import FFMPEGEncoder
import sys
import ffmpeg_bindings
from rclpy.serialization import serialize_message
from pathlib import Path
sys.path.append("/home/ubuntu/densha/infrabel_ros2_ws/utils")
from rosbag import BagFileParser  # noqa: E402


encoder = FFMPEGEncoder() # create encoder object

rosbag_path = Path("/data/internal/infrabel_data/via_run_infrabel_2023-09-18_12-58-54/via_run_infrabel_2023-09-18_12-58-54_0.db3")

bag = BagFileParser(rosbag_path)


msg = ffmpeg_bindings.cpp2py_img(10, 10)
print(ffmpeg_bindings.cpp2py_ffmpeg(10, 10))
topic = '/deg120/image'
encoder.setParameters("libx264", "", "", "", 10, 8242880, 0, "")
encoder.initialize(1920, 1080)
timestamps  = bag.get_timestamps(topic)
for k, timestamp in enumerate(timestamps):
    
    msg = bag.get_message_for_timestamp(topic, timestamp)
    print(type(msg.header))
    
    
    out = encoder.encodeImage(msg)
    print(type(out))
