python src/lerobot/scripts/lerobot_record_HIL_joint_delta.py  \
    --robot.type=piper_follower \
    --robot.port=can_follower \
    --robot.cameras="{ \
        camera1: {type: opencv, index_or_path: '/dev/video5', width: 640, height: 480, fps: 30},
        camera2: {type: opencv, index_or_path: '/dev/video11', width: 640, height: 480, fps: 30}}" \
    --robot.id=follower \
    --teleop.type=piper_leader \
    --teleop.port=can_leader \
    --teleop.id=leader \
    --display_data=true \
    --dataset.repo_id=your_HF_id/your_repo_id \
    --dataset.num_episodes=10 \
    --dataset.single_task="Place the left block on the right cup." \
    --policy.path=outputs/pretrain/checkpoints/last/pretrained_model \
    --dataset.episode_time_s=60