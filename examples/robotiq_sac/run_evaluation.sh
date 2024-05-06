export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.3 && \
python sac_policy_robotiq.py "$@" \
    --actor \
    --env robotiq-grip-v1 \
    --exp_name=sac_robotiq_policy_evaluation \
    --eval_checkpoint_path "/home/nico/real-world-rl/serl/examples/robotiq_sac/checkpoints 0411-16:46"\
    --eval_checkpoint_step 100000 \
    --eval_n_trajs 10 \
    --debug
