lerobot-record  \
    --robot.type=piper_follower \
    --robot.port=can_follower \
    --robot.cameras="{ \
        camera1: {type: opencv, index_or_path: '/dev/video4', width: 640, height: 480, fps: 30},
        camera2: {type: opencv, index_or_path: '/dev/video0', width: 640, height: 480, fps: 30}}" \
    --robot.id=follower   \
    --teleop.type=piper_leader \
    --teleop.port=can_leader \
    --teleop.id=leader \
    --dataset.repo_id=your_HF_id/your_repo_id \
    --dataset.num_episodes=10 \
    --dataset.single_task="Put the bell pepper in the right cup." \
    --dataset.episode_time_s=90
