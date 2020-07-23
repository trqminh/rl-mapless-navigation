import matplotlib.pyplot as plt
import os
import csv
from pathlib import Path


def main():
    for item in ['bl-env1/', 'bl-env2/', 'vis_obs-env2/']:
        results = []
        dir_name = './figures/' + item
        file_name = dir_name + 'avg_reward_his.csv'

        with open(file_name, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                results.append(float(row[0]))

        train_time_step = [i*1. for i in range(10000, 10000*len(results) + 1, 10000)]
        plt.plot(train_time_step, results, label=item)

        plt.ylabel(item)
        plt.xlabel('Training time step')
        plt.legend(loc='lower right')
        plt.savefig(dir_name + 'avg_reward_his.png')
        plt.clf()


if __name__=='__main__':
    main()
