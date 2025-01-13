export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.3 && \
python sac_policy.py "$@" \
    --actor \
<<<<<<< HEAD:examples/robotiq_sac/run_evaluation.sh
    --env robotiq_basic_env \
    --exp_name=sac_robotiq_policy_evaluation \
    --eval_checkpoint_path "/home/andrea/Code/Thesis/voxel-serl/examples/robotiq_sac/checkpoints 1016-15:38"\
    --eval_checkpoint_step 30000 \
=======
    --env box_picking_basic_env \
    --exp_name=sac_drq_policy_evaluation \
    --eval_checkpoint_path "/home/nico/real-world-rl/serl/examples/box_picking_sac/checkpoints 0411-16:46"\
    --eval_checkpoint_step 100000 \
>>>>>>> upstream/develop:examples/box_picking_sac/run_evaluation.sh
    --eval_n_trajs 10 \
    --debug
