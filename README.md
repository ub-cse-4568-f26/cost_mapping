# Robot Perception - Cost Mapping

***This class activity is to be done as individuals, not with partners nor with teams.***

### How to get started →

Begin by reading this entire writeup and making sure you have a good understanding of it. Next, spend some time planning how you’re going to approach the problem. 

## Introduction

---

In this assignment, you will implement SLAM based mapping and cost-map generation based on algorithms provided to you. The assignment is divided in two parts, for the 1st part you will setup, build and run a popular LiDAR SLAM pipeline to generate a dense point cloud map. In part 2 you will convert this point cloud into a cost map which can then be used for obstacle avoidance and shortest path planning. 




## Activity Information

---

### Objectives

- Learn how to build an run large SLAM pipelines
- Implement Cost-mapping and generate occupancy maps
- Test the implementation on a dataset of LiDAR scans
  

### Resources

- [Quick Introduction to SLAM](https://ouster.com/insights/blog/introduction-to-slam-simultaneous-localization-and-mapping)
- [FAST-LIO2](https://arxiv.org/abs/2107.06829)
- [Occupancy Grid](https://docs.ros.org/en/humble/p/nav_msgs/msg/OccupancyGrid.html)

### Requirements

- Your ROS2 package should run without errors
- Your code should be well documented
- Your code should NOT download the dataset from the internet (we will have a copy of the dataset)

  
### What we provide

- Detailed instructions to download the dataset and run FAST-LIO2
- Dev PLY for testing purposes
- A sample cost map generated using the algorithm described

### FAST-LIO2

FAST-LIO2 is a LiDAR SLAM system that uses LiDAR and IMU data in a tightly-coupled manner for pose estimation and mapping. The system generates high-resolution point cloud maps such as the one shown below.

![alt text](images/image.png)

Image shows point cloud map of Davis hall atrium generated using the Spot robot and a Ouster LiDAR using FAST-LIO2.

You can start with setting up FAST-LIO2 in your ROS environment. The code and detailed instructions are provided [here](https://github.com/ub-cse-4568-f25/fastlio2)

**Note: Make sure to read the entire README and follow all the steps. You should NOT have to make any changes to the source code. **


### The Datasets

We provide datasets that you will use to generate point clouds using FAST-LIO2. The datasets can be downloaded here in the form of `rosbags2`. The ros2 bags contain various topics among which you'll find `/ouster/points` and `/ouster/imu` which are of type `sensor_msgs/Pointcloud2` and `sensor_msgs/Imu` respectively. The `ros2 bag` can be downloaded [here](https://buffalo.box.com/s/ye2azyb62p9eovxio685o6si725kk0cz)

Testing ply can be downloaded from [here](https://buffalo.box.com/s/oztatbku5zjyqgc1jh1cu9bx8u7e0hhg)


### What to submit

You must submit a package with the following file structure


```bash
cost_mapping/
├── config
│   └── params.yaml #Your params go here
├── part1_result.png # your part1 screenshot
├── cost_mapping
│   ├── __init__.py
│   ├── occupancy_pub.py # Node to publish occupancy grid
│   └── utils.py    # node which contains cost_map function
├── images
│   ├── algo.png
│   ├── costmap.png
│   ├── image-1.png
│   ├── image.png
│   ├── occupancy.png
│   ├── scene.png
│   ├── top_view2.png
│   └── top_view.png
├── launch
│   └── occupancy.launch.py # Should launch your node and occupancy grid in rviz
├── package.xml
├── README.md
├── resource
│   └── cost_mapping
├── rviz
│   └── config.rviz # rviz configuration
├── setup.cfg
├── setup.py    # update for required installs
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py

```

Your code should accept parameters from the `params.yaml` file. The file should contain the following fields:

```yaml
point_cloud_path: xyz.pcd # Path to the pcd file
elevation_thres_max: 10 # 3 for development
elevation_thres_min: -0.5
nearest_neighbors: 30
cost_thres: 0.5 # Normalized cost between 0-1
occupancy_topic: occupancy_grid
image_size: 600
```

*Please make sure you adhere to the structure above, if your package doesn’t match it the grader will give you a **zero***

### Grading considerations

- **Late submissions:** Carefully review the course policies on submission and late assignments. Verify before the deadline that you have submitted the correct version.
- **Environment, names, and types:** You are required to adhere to the names and types of the functions and modules specified in the release code. Otherwise, your solution will receive minimal credit.

## Part 1 : Point Cloud Mapping

---

In this part, you will generate point cloud maps using FAST-LIO2 and the dataset provided. The point cloud should be in `.pcd` and should be complete (make sure to run the entire bag). Do Not Upload the generated pointcloud (.pcd)

### Suggested Plan Of Attack

1. Start by setting up FAST-LIO2. For this assignment, we will be using MIT-SPARK/spark-fast-lio and the [Colloseo dataset](https://buffalo.box.com/s/ye2azyb62p9eovxio685o6si725kk0cz). You should use the same launch procedure as [LIO on Colloseo](https://github.com/ub-cse-4568-f25/fastlio2?tab=readme-ov-file#%EF%B8%8F-lio-on-the-colosseum) but on the dataset provided to you.
2. Run FAST-LIO2 and play the bag using `ros2 bag play <name>`.
3. Visualize results in `rviz2` (FAST-LIO2 launch will start a `rviz2` window)
4. On completion the point cloud map will be saved in `scene.pcd` in the folder where you launch FAST-LIO2
5. Inspect the point cloud using [CloudCompare](https://www.danielgm.net/cc/) or PCL Tools

Submit a screenshot called `part1_result.png` containing the CloudCompare scene for evaluation.


## Part 2 : Cost Mapping

---
![alt text](images/image-1.png)

> The first figure shows the 3D scan taken by a LiDAR onboard a robot. The second figure shows a binary cost map with traversable region having a value of 0 and non-traversable region having a value of 1. A hyper-parameter determines traversability. The third figure show a real-valued cost map. Here each pixel has a finite value which can be used to compute realistic costs

Once you have your point cloud map and want to use it for robot navigation, you need to convert it into a cost map. A cost map in robotics is a representation of the environment that includes information about potential costs associated with traversing to different locations. This information helps robots make informed decisions and plan efficient paths to achieve their goals. You will implement a cost map algorithm that takes into account the gradient of elevation to assign cost. 

### Cost Mapping Algorithm

![](images/algo.png)

1. Read the PCD using open3d (you can refer to the ICP activity). You can specify your path in`params.yaml` during testing using `point_cloud_path`, but should be mapped to `scene.pcd` during submission. The code should be such that it takes the path from `point_cloud_path` relative to the home(`~`) directory. i.e., if your home directory is `/home/user`, the path to `scene.pcd` is `/home/user/scene.pcd`.
2. Convert the open3d format to a numpy array (open3d has a function for this)
3. T is tree data structure. This drastically speeds up your nearest neighbor searches. We recommend that you use the KDTree from Scipy
4. The median function should calculate the median value in the passed list
5. The normalize function should normalize all the values in the cost list to be between 0-1
6. Iterate through each point in tree, calculate its pixel coordinate and create a cost map from these pixels using the normalized cost as the intensity value

Inside `cost_mapping.py` write your logic to compute the 2D cost map. You can reuse this code in the ROS portion of the assignment. **We will check the functionality and correctness of this code for Part 2-i.**

```python
def cost_map(<your args>) -> numpy.ndarray:
    """
    Calculate the costmap for a ply file.
    Args:
        <your args>
    Returns:
        numpy.ndarray: The costmap image for the ply file.
    """

# Your code must return a 2D image and should be plottable with matplotlib.
```

**When run on the dev_workspace.ply file, the output is as follows**

**Top View**
![](images/scene.png)


**Cost Map**
![](images/costmap.png)



### Occupancy Grid

After successfully generating a cost map, we can simplify it by thresholding the cost to generate a occupancy grid. Meaning, we will set a certain cost above which the terrain/region is not traversable for the robot and below which it is. The occupancy grid can then be used for graph-based path planning.

You will write a ROS node `occupancy` which will take a static point cloud map (from FAST-LIO2), use your cost mapping algorithm to generate a cost map, threshold it and publish an occupancy grid (`nav_msgs/OccupancyGrid`)/

**Occupancy Grid**
![](images/occupancy.png)

### Suggested Plan of Attack

1. Downsample the pointcloud with a voxel size of 0.01
2. Implement the cost mapping algorithm in `cost_mapping.py` as a function you can call
3. Write your ROS2 Node that imports `cost_mapping.py` and calls your `cost_map` function to generate cost map. You can use the sharedpath of the package and sys.path.append to import the `cost_mapping` module
4. Threshold the cost map to generate an occupancy
5. Publish occupancy grid
6. Make sure to add the necessary launch, rviz, param configurations and the corresponding folders in the data files in setup.py and 

# Submission and Assessment

---

Submit using the Github upload feature on [autolab](https://autolab.cse.buffalo.edu)

**Note: Make sure your code complies to all instructions, especially the naming conventions. Failure to comply will result in zero credit**

You will be graded on the following. Penalties are listed under each point, absolute values, w.r.t activity total.

1. Part 1 (Point Cloud Mapping) [15%] 
2. Part 2 (Cost Mapping) [85%]
    1. Correctness of cost-map algorithm [45%]
    2. Publishing occupancy grid [40%]

**Note: You will not receive partial credit for files working /not working. Please make sure your scripts execute**
