CLAMP: Combined Learning from Demonstration and Motion Planning
===================================================
CLAMP is an probabilistic inference based approach which learns a a trajectory prior from human demonstrations and uses factor graph optimization for efficient skill reproduction. The baseline capabilities of CLAMP are described in [Towards robust skill generalization: Unifying learning from demonstration and motion planning](http://proceedings.mlr.press/v78/rana17a/rana17a.pdf) (CoRL 2017). Extensions of CLAMP to learn from demonstrations provided in clutter as well as incrementally updating the trajectory prior are provided in [Learning Generalizable Robot Skills from Demonstrations in Cluttered Environments](https://arxiv.org/pdf/1808.00349.pdf) (IROS 2018). 

This library is an implementation of CLAMP. The core library is developed in C++ language, alongside a MATLAB frontend. Demo scripts are provided in MATLAB to reproduce some of the results in our publications.

CLAMP is being developed by [M. Asif Rana](mailto:asif.rana@gatech.edu) and [Mustafa Mukadam](mailto:mmukadam3@gatech.edu) at the Georgia Tech Institute for Robotics & Intelligent Machines.

Prerequisites
------

- CMake >= 2.6 (Ubuntu: `sudo apt-get install cmake`), compilation configuration tool.
- [Boost](http://www.boost.org/) >= 1.50 (Ubuntu: `sudo apt-get install libboost-all-dev`), portable C++ source libraries.
- [GTSAM](https://bitbucket.org/gtborg/gtsam) >= 4.0 alpha, a C++ library that implement smoothing and mapping (SAM) framework in robotics and vision.
Here we use factor graph implementations and inference/optimization tools provided by GTSAM.
- [GPMP2](https://github.com/gtrll/gpmp2), a C++ library that implements Gaussian Process Motion Planner 2.

(To use all the features, install the MATLAB toolboxes for GTSAM and GPMP2)

Compilation & Installation
------

In the library folder excute:

```
$ mkdir build
$ cd build
$ cmake ..
$ make check  # optonal, run unit tests
$ make install
```


Matlab Toolbox
-----

A Matlab toolbox is provided to use our library in Matlab. To enable Matlab toolbox during compilation:

```
$ cmake -DCLAMP_BUILD_MATLAB_TOOLBOX:OPTION=ON -DGTSAM_TOOLBOX_INSTALL_PATH:PATH=/path/install/toolbox ..
$ make install
```

After you install the Matlab toolbox, don't forget to add `/path/install/toolbox` to your Matlab path.

Usage
-----

The MATLAB toolbox contains multiple demo scripts described as follows,

1. `mainBatchCLAMP.m` - learns trajectory prior from an already acquired dataset, followed by skill generalization [1](http://proceedings.mlr.press/v78/rana17a/rana17a.pdf).
2. `mainIncrementalCLAMP.m` -  updates the prior as dataset is aggregated, followed by skill generalization [2](https://arxiv.org/pdf/1808.00349.pdf).
3. `mainBatchCLAMP_ObstacleWeightedPrior.m` - learns prior from demonstrations provided in the presence of clutter [2](https://arxiv.org/pdf/1808.00349.pdf).


Citing
-----

If you use CLAMP in an academic context, please cite the following publications:

```
[1] 
@inproceedings{rana2017towards,
  title={Towards robust skill generalization: Unifying learning from demonstration and motion planning},
  author={Rana, Muhammad Asif and Mukadam, Mustafa and Ahmadzadeh, S Reza and Chernova, Sonia and Boots, Byron},
  booktitle={Proceedings of the 2017 Conference on Robot Learning (CoRL)},
  year={2017}
}

[2]
@inproceedings{rana2018learning,
  title={Learning Generalizable Robot Skills from Demonstrations in Cluttered Environments},
  author={Rana, M Asif and Mukadam, Mustafa and Ahmadzadeh, S Reza and Chernova, Sonia and Boots, Byron},
  booktitle={2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4655--4660},
  year={2018},
  organization={IEEE}
}

```

Questions & Bug reporting
-----

Please use Github issue tracker to report bugs. For other questions please contact [Asif Rana](mailto:asif.rana@gatech.edu)
or [Mustafa Mukadam](mailto:mmukadam3@gatech.edu) .


License
-----

CLAMP is released under the BSD license, reproduced in the file LICENSE in this directory.
