export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.3 && \
python bc_policy_robotiq.py "$@" \
    --env robotiq_basic_env \
    --exp_name=bc_robotiq_policy \
    --seed 42 \
    --batch_size 256 \
    --demo_paths robotiq_test_20_demos_2024-10-08_11-24-34.pkl \
    --eval_checkpoint_step 0
    # --debug # wandb is disabled when debug
