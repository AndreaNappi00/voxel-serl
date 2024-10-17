export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.3 && \
python sac_policy_robotiq.py "$@" \
    --actor \
    --env robotiq_basic_env \
    --exp_name=sac_robotiq_policy_evaluation \
    --eval_checkpoint_path "/home/andrea/Code/Thesis/voxel-serl/examples/robotiq_sac/checkpoints 1016-15:38"\
    --eval_checkpoint_step 30000 \
    --eval_n_trajs 10 \
    --debug
