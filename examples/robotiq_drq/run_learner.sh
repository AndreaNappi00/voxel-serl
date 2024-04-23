export XLA_PYTHON_CLIENT_PREALLOCATE=false && \
export XLA_PYTHON_CLIENT_MEM_FRACTION=.2 && \
python drq_policy_robotiq.py "$@" \
    --learner \
    --env robotiq_camera_env \
    --exp_name=drq_robotiq_policy \
    --seed 0 \
    --random_steps 100 \
    --training_starts 100 \
    --utd_ratio 4 \
    --batch_size 512 \
    --eval_period 1000 \
    --encoder_type resnet-pretrained \
    --checkpoint_period 10000 \
    --demo_path TODO \