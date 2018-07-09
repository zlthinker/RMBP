## RMBP

Robust Matching using Belief Propagation (RMBP) is a robust matcher used to reject outliers from 3D point matches. 

It is proposed by the ECCV 2018 paper:

"Learning and Matching Multi-View Descriptors for Registration of Point Clouds", 
Lei Zhou, Siyu Zhu, Zixin Luo, Tianwei Shen, Runze Zhang, Mingmin Zhen, Tian Fang, Long Quan.

## Usage

### Dependencies
* Cmake
* OpenMP

### Building
Build like any other cmake project.

```
mkdir build & cd build
cmake ..
make -j
```

### Run test code
In the `test` folder are a list of match files for demo. They first write the match number `N`, followed by `N` lines that denote `x`, `y`, `z` of the two corresponding points in two point clouds and the initial probability of being an inlier for the match pair which may be estimated from their measurements like similarity.

The first 150 match pairs are inliers while the others are outliers. The demo outputs the belief of being an inlier for each match pair and the refined match pairs given the belief threshold.

```
cd build
./RMBP ../test/match_file_inlier_ratio_4.txt 0.45
```
