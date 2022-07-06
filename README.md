# DriftReducingNeuralNetwork
# This repository shows a Matlab implementation for a drift reducing algorithm published in the paper titled "Drift Reduction for Monocular Visual Odometry of Intelligent Vehicles using Feedforward Neural Networks" with the following abstract 
"In this paper, an approach for reducing the drift in monocular visual odometry algorithms is proposed based on a feedforward neural network. A visual odometry algorithm computes the incremental motion of the vehicle between the successive camera frames, then integrates these increments to determine the pose of the vehicle.
The proposed neural network reduces the errors in the pose estimation of the vehicle which results from the inaccuracies in features detection and matching, camera intrinsic parameters, and so on. These inaccuracies are propagated to the motion estimation of the vehicle causing larger amounts of estimation errors.
The drift reducing neural network identifies such errors based on the motion of features in the successive camera frames leading to more accurate incremental motion estimates.
The proposed drift reducing neural network is trained and validated using the KITTI dataset and the results show the efficacy of the proposed approach in reducing the errors in the incremental orientation estimation, thus reducing the overall error in the pose estimation."
IN order to run the algorithm please download the KITTI camera odometry data set 
from http://www.cvlibs.net/datasets/kitti/eval_odometry.php and choose gray scale data. 
1-Load the data set into your Matlab workspace. 
2-load all the matlab codes in this repository in your workspace. 
3-run the "ImageMonoProcess.m" code. 
4-if its desired to train a new NN save the SEQXXL & SEQXXT for the desired 60% of the training sequences and use the "NNTrainingScript.m" for training, some training data are already available in this repository in "traindata.mat" also an already trained NN is availed in "newNN.mat". 
5- after acquiring the NN run "ImageMonoProcess.m" code again for the specified sequence then load the training NN into the workspace. 
6- Run "NNDataProcessing.m". 
7- Run "ResultsPlotting.m" which will plot the remaining 40% that are not included in the NN training. 
Please note that "newNN.mat" is not trained on any of seq 10 data since its used completely for testing in the paper.
this work has been tested on an ASUS K501UX laptop with 8 GB of ram, Nvidia GTX 950M graphics card and an intel core I7-6500U processor.
