import matplotlib.pyplot as plt
import os
import csv
from pathlib import Path


def main():
    save_path = './figures/comp_imgs/'
    Path(save_path).mkdir(parents=True, exist_ok=True)

    env_id = 2
    compare_items = ['bl-env', 'vis_obs-env']
    compare_items_name = ['Phương pháp cơ sở', 'Phương pháp đề xuất']
    figure_name = 'avg_reward_his'

    for idx, item in enumerate(compare_items):
        results = []
        dir_name = './figures/' + item + str(env_id) + '/'
        file_name = dir_name + figure_name + '.csv'

        with open(file_name, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                results.append(float(row[0]))

        train_time_step = [i*1. for i in range(10000, 10000*len(results) + 1, 10000)]
        plt.plot(train_time_step, results, label=compare_items_name[idx])

    plt.ylabel('Phần thưởng trung bình')
    plt.xlabel('Số bước huấn luyện')
    plt.legend(loc='lower right')
    plt.savefig(save_path + 'comp_avg_reward_his_' +str(env_id) + '.png')
    plt.clf()


if __name__=='__main__':
    main()
