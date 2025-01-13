export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.3 && \
python bc_policy.py "$@" \
    --env box_picking_basic_env \
    --exp_name=bc_drq_policy \
    --seed 42 \
    --batch_size 256 \
<<<<<<<< HEAD:examples/robotiq_bc_spacemouse/run_rbc_learner.sh
    --demo_paths robotiq_test_20_demos_2024-10-08_11-24-34.pkl \
========
    --demo_paths "" \
>>>>>>>> upstream/develop:examples/box_picking_bc/run_rbc_learner.sh
    --eval_checkpoint_step 0
    # --debug # wandb is disabled when debug
