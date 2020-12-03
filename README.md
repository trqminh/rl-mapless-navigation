# Mobile Robot Planner with Low-cost Cameras Using Deep Reinforcement Learning

## Abstract
This work attempts to construct pseudo laser findings based on direct depth matrix prediction from a single camera image while still retaining the stable performance of the robot navigation task. The depth prediction phase is inherited from [monodepth2](https://github.com/nianticlabs/monodepth2) pre-trained on the KITTI dataset.   

The paper can be found [here]().   

## Demo
<img src="/misc/demo.gif" class="fit image">   

It can be seen that the robot on the right-hand side moves surprisingly faster. However, the reason is that at each time step, the policy accompanied by a visual prediction module obviously takes more time to make a decision. Therefore, when a decision is not yet made, the robot moves on inertia.

## Installation
This repo contains the [trained models](./src/trained_models/) to perform like the demo above. It is also daunting to reproduce this result. The reason could be briefly explained in this [article](https://www.alexirpan.com/2018/02/14/rl-hard.html). See the [installation](./INSTALL.md) for more information.

## Acknowledgement
- [MotionPlannerUsingDDPG](https://github.com/m5823779/MotionPlannerUsingDDPG)
- [Lei Tai et al.](https://arxiv.org/pdf/1703.00420.pdf)
- [monodepth2](https://github.com/nianticlabs/monodepth2)

