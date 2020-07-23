#!/usr/bin/env python3
import rospy
#import gym
import numpy as np
import tensorflow as tf
from ddpg import *
from environment import Env
from pathlib import Path
import argparse

exploration_decay_start_step = 50000
state_dim = 16
action_dim = 2
action_linear_max = 0.25  # m/s
action_angular_max = 0.5  # rad/s

def write_to_csv(item, file_name):
    with open(file_name, 'a') as f:
        f.write("%s\n" % item)

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--train', type=int, default=0, help='1 for training and 0 for testing')
    parser.add_argument('--env_id', type=int, default=2, help='env name')
    parser.add_argument('--sac', type=int, default=0, help='1 for using sac')
    parser.add_argument('--visual_obs', type=int, default=0, help='1 for using image at robot observation')
    parser.add_argument('--test_env_id', type=int, default=2, help='test environment id')
    parser.add_argument('--n_scan', type=int, default=10, help='num of scan sampled from full scan')

    args = parser.parse_args()
    return args

def main():
    rospy.init_node('baseline')

    # get arg
    args = parse_args()
    is_training = bool(args.train)
    env_name = 'env' + str(args.env_id)
    trained_models_dir = './src/trained_models/bl-' + env_name + '-models/' if not args.visual_obs else \
            './src/trained_models/vis_obs-' + env_name + '-models/'

    env = Env(is_training, args.env_id, args.test_env_id, args.visual_obs, args.n_scan)
    agent = DDPG(env, state_dim, action_dim, trained_models_dir)
    past_action = np.array([0., 0.])
    print('State Dimensions: ' + str(state_dim))
    print('Action Dimensions: ' + str(action_dim))
    print('Action Max: ' + str(action_linear_max) + ' m/s and ' + str(action_angular_max) + ' rad/s')

    if is_training:
        print('Training mode')
        # path things
        figures_path = './figures/bl-' + env_name + '/' if not args.visual_obs else \
            './figures/vis_obs-' + env_name + '/'

        Path(trained_models_dir + 'actor').mkdir(parents=True, exist_ok=True)
        Path(trained_models_dir + 'critic').mkdir(parents=True, exist_ok=True)
        Path(figures_path).mkdir(parents=True, exist_ok=True)

        avg_reward_his = []
        total_reward = 0
        var = 1.
        ep_rets = []
        ep_ret = 0.

        while True:
            state = env.reset()
            one_round_step = 0

            while True:
                a = agent.action(state)
                a[0] = np.clip(np.random.normal(a[0], var), 0., 1.)
                a[1] = np.clip(np.random.normal(a[1], var), -0.5, 0.5)

                state_, r, done, arrive = env.step(a, past_action)
                time_step = agent.perceive(state, a, r, state_, done)

                if arrive:
                    result = 'Success'
                else:
                    result = 'Fail'

                if time_step > 0:
                    total_reward += r
                    ep_ret += r

                if time_step % 10000 == 0 and time_step > 0:
                    print('---------------------------------------------------')
                    avg_reward = total_reward / 10000
                    print('Average_reward = ', avg_reward)
                    avg_reward_his.append(round(avg_reward, 2))
                    print('Average Reward:',avg_reward_his)
                    total_reward = 0
                    print('Mean episode return over training time step: {:.2f}'.format(np.mean(ep_rets)))
                    print('Mean episode return over current 10k training time step: {:.2f}'.format(np.mean(ep_rets[-10:])))
                    write_to_csv(np.mean(ep_rets), figures_path + 'mean_ep_ret_his.csv')
                    write_to_csv(np.mean(ep_rets[-10:]), figures_path + 'mean_ep_ret_10k_his.csv')
                    write_to_csv(avg_reward, figures_path + 'avg_reward_his.csv')
                    print('---------------------------------------------------')

                if time_step % 5 == 0 and time_step > exploration_decay_start_step:
                    var *= 0.9999

                past_action = a
                state = state_
                one_round_step += 1

                if arrive:
                    print('Step: %3i' % one_round_step, '| Var: %.2f' % var, '| Time step: %i' % time_step, '|', result)
                    one_round_step = 0
                    if time_step > 0:
                        ep_rets.append(ep_ret)
                        ep_ret = 0.

                if done or one_round_step >= 500:
                    print('Step: %3i' % one_round_step, '| Var: %.2f' % var, '| Time step: %i' % time_step, '|', result)
                    if time_step > 0:
                        ep_rets.append(ep_ret)
                        ep_ret = 0.
                    break

    else:
        print('Testing mode')
        total_return = 0.
        total_step = 0
        while True:
            state = env.reset()
            one_round_step = 0

            while True:
                a = agent.action(state)
                a[0] = np.clip(a[0], 0., 1.)
                a[1] = np.clip(a[1], -0.5, 0.5)
                state_, r, done, arrive = env.step(a, past_action)
                total_return += r
                past_action = a
                state = state_
                one_round_step += 1
                total_step += 1


                if arrive:
                    print('Step: %3i' % one_round_step, '| Arrive!!!')
                    one_round_step = 0
                    if env.test_goals_id >= len(env.test_goals):
                        print('Finished, total return: ', total_return)
                        print('Total step: ', total_step)
                        exit(0)

                if done:
                    print('Step: %3i' % one_round_step, '| Collision!!!')
                    break


if __name__ == '__main__':
     main()
