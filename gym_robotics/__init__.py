from gym.envs.registration import register

# Standard Environments
for reward_type in ['dense', 'sparse', 'very_sparse']:
    if reward_type == 'dense':
        suffix = 'Dense'
    elif reward_type == 'sparse':
        suffix = 'Sparse'
    else:
        suffix = 'VerySparse'
    kwargs = {
        'reward_type': reward_type,
    }

    # Fetch
    register(
        id='FetchSlide{}-v2'.format(suffix),
        entry_point='gym_robotics.envs.mujoco:FetchSlideEnv',
        kwargs=kwargs,
        max_episode_steps=50,
    )

    register(
        id='FetchPickAndPlace{}-v2'.format(suffix),
        entry_point='gym_robotics.envs.mujoco:FetchPickAndPlaceEnv',
        kwargs=kwargs,
        max_episode_steps=50,
    )

    register(
        id='FetchReach{}-v2'.format(suffix),
        entry_point='gym_robotics.envs.mujoco:FetchReachEnv',
        kwargs=kwargs,
        max_episode_steps=50,
    )

    register(
        id='FetchPush{}-v2'.format(suffix),
        entry_point='gym_robotics.envs.mujoco:FetchPushEnv',
        kwargs=kwargs,
        max_episode_steps=50,
    )
