# Knowledge Aggregation in Multi-Agent Self-Adaptive Cyber-Physical Systems

This repository contains the source code and resources to perform Knowledge Aggregation with various probabilistic logics, such as Subjective Logic, in a ROS-based Simulated Self-Adaptive Multi-Robot System.

## Important note
This chapter was added to this readme since the initial pure simulator code was changed and this needs to be described here. Responsible for these changes (and the causing evaluation framework) is Sebastian Bergemann. Questions about everything else should be directed to Malte Neuss, who is currently the maintainer of the actual simulator.

### The purpose of the changes (and this branch)
This branch contains the version which can be used together with an evaluation framework.
The purpose of this evaluation framework is to be able to better evaluate and maybe compare different simulators like the one mentioned above. The evaluation framework contains also a wrapper, which should make it easier to handle such simulators without deep knowledge of them. It collects all important parameters and input possibilities (like map, dirt distribution, etc.), which can easy be change, and can launch the simulator with one command while providing all needed inputs.

Since the current simulators were mainly made before the evaluation framework, they have not the same input and output interfaces. So, the framework and especially the wrapper introduce a more generic interface, which each simulator needs to adapt to. As a result a modified version of each simulator was created and you are currently inside the one for this simulator.

If you are only interested in the pure simulator, you might want to go to the master branch, otherwise I will explain now how to use the evaluation framework with this simulator.

### Installing the simulator with the evaluation framework
The first step is to install the actual simulator. For this purpose, you should follow the instructions of Malte Neuss, who split them up in the ROS part and the SL part. Start with the next chapter, where this is explained. Some notes on that: 
- At some point you will be asked to clone this repository. Do it, but do not forget that the cloned repository will be still on the master branch. Since you want to have the changed version with the evaluation, you need to switch the git branch after cloning, e.g. 
```shell
git checkout with_eval_framework           #or however this branch is called
```
- If you are instructed to execute `catkin_make`, do it, but it is likely that it will fail since in this changed branch some packages have dependencies on the `wrapper-msgs`, which you will not have until the installation of the evaluation framework in the next step. Just go on (you will execute it later again).


If you have installed it, you can continue with adding the [evaluation framework](https://gitlab.lrz.de/master-thesis-bergemann/evaluation-framework-code). The whole repository needs to be cloned into the [source folder](../ros/src) of this simulator workspace like it is just another ROS package. When you are inside the source folder, you can get it with:

```shell
git clone git@gitlab.lrz.de:master-thesis-bergemann/evaluation-framework-code.git 
```

Afterwards you should build the workspace again since new ROS packages and nodes are introduced. In the top of the catkin workspace ([ros](../ros) folder, where the `src` folder is located), execute (like you should have done already during the simulator installation):

```shell
catkin_make 
```

In the end, follow the installation instructions of the readme at the top level of the new cloned `evaluation-framework-code` folder. It should actually be quite simply. You only need to specify where the simulator can be launched and then check if the settings are like you want.

For this simulator I can already say that the required simulator launch file is the `master.launch` in the [test](../ros/test) folder. You need to specify it in `parameters.yaml` (inside `evaluation-framework-code/launchers/config`) by adding them behind the `absolute_path_to_simulator_launch` attribute (with your local absolute path). The option of describing it relative with a ROS package and the internal path is not possible for this simulator since Malte located the launch file outside of all existing ROS packages. You might also want to set some internal simulator parameters by specifying them in the same YAML file with for example:

```yaml
sim_external_arguments: "use_sl:=false sl_operator:=Comb false_positive:=false false_negative:=false adaptive_scheduling:=true"
```

Now, continue with the normal simulator readme:

## Workspace Structure

The [ros](../ros) folder contains the ROS implementation of a Multi-Robot system. This ROS-based testbed consists of a set of Python and C++ ROS nodes that allow for an easy experimentation with the knowledge aggregation using the different available fusion operators in subjective logic.

The [subjective_logic](../subjective_logic) directory contains the source code that implements the formalisms of subjective logic; in particular, it provides an API to perform the aggregation of the knowledge of multiple agents using different fusion operators. 

The folders themselves contain a more detailed explanation about their content. In particular, the [ros](../ros) folder incorporates a detailed step by step guide on how to install and run the system, also including architectural details of the system.
