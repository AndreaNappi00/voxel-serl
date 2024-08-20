export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.3 && \
python /home/nico/real-world-rl/serl/examples/robotiq_drq/drq_policy_robotiq.py "$@" \
    --learner \
    --env robotiq_camera_env \
    --exp_name="voxnet only" \
    --camera_mode pointcloud \
    --max_traj_length 100 \
    --seed 1 \
    --max_steps 20000 \
    --random_steps 0 \
    --training_starts 500 \
    --utd_ratio 8 \
    --batch_size 128 \
    --checkpoint_period 2500 \
    --checkpoint_path /home/nico/real-world-rl/serl/examples/robotiq_drq/experiment_setup/VoxNet/checkpoints \
    --demo_path /home/nico/real-world-rl/serl/examples/robotiq_drq/experiment_setup/box_picking_20_demos_2024-08-20_voxelgrid.pkl \
    \
    --encoder_type voxnet \
    --state_mask all \
    --encoder_bottleneck_dim 128 \
#    --debug
