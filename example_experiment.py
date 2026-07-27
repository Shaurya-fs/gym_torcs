from gym_torcs import TorcsEnv
from autom_agent import AutonomousAgent
import numpy as np

vision = True
episode_count = 10
max_steps = 50

reward = 0
done = False
step = 0

# Create TORCS environment
env = TorcsEnv(
    vision=vision,
    throttle=False
)

# Create my autonomous racing agent
agent = AutonomousAgent()

print("===================================")
print("Autonomous Racing Experiment Start")
print("===================================")

for episode in range(episode_count):

    print(f"\nEpisode {episode + 1}/{episode_count}")

    if episode % 3 == 0:
        observation = env.reset(relaunch=True)
    else:
        observation = env.reset()

    total_reward = 0

    for _ in range(max_steps):

        try:
            action = agent.act(
                observation,
                reward,
                done,
                vision,
            )

            observation, reward, done, _ = env.step(action)

            total_reward += reward
            step += 1

            if done:
                break

        except Exception as e:
            print("\nController Error")
            print(e)
            break

    print(f"Episode Reward : {total_reward}")
    print(f"Total Steps    : {step}")

env.end()

print("\nExperiment Finished.")