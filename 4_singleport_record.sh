# Wired leader->follower on ONE CAN bus. The PC only listens (follower runs in passive mode).
# action = leader control frames seen on the bus (--action_source=leader_command, default).
# Check first that those frames are visible:  python examples/piper/piper_check.py monitor --port can0
python src/lerobot/scripts/lerobot_record_singleport.py \
  --robot.type=piper_follower \
  --robot.port=can0 \
  --robot.id=follower \
  --action_source=leader_command \
  --dataset.repo_id=your_HF_id/your_repo_id \
  --dataset.single_task="Pick up the ball and place it in the drop zone." \
  --dataset.num_episodes=10 \
  --dataset.episode_time_s=60 \
  --dataset.reset_time_s=10 \
  --dataset.fps=10 \
  --display_data=true
