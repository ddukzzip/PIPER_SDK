lerobot-teleoperate \
    --robot.type=piper_follower \
    --robot.port=can_follower \
    --robot.cameras="{ \
    camera1: {type: opencv, index_or_path: '/dev/video4', width: 640, height: 480, fps: 30},
    camera2: {type: opencv, index_or_path: '/dev/video10', width: 640, height: 480, fps: 30}
    }" \
    --robot.id=follower \
    --teleop.type=piper_leader \
    --teleop.port=can_leader \
    --teleop.id=leader \
    --display_data=true
