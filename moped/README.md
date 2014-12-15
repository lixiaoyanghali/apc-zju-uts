It recognizes objects from point-based features (e.g. SIFT, SURF) and their geometric relationships extracted from rigid 3D models of objects. The global MOPED framework requires seven steps to recognize objects: 

1) Feature extraction

2) Feature matching

3) Feature clustering

4) Hypothesis generation

5) Pose clustering

6) Hypothesis refinement

7) Pose recombination 

This code is structured in two modules: first, a ROS-agnostic library called libmoped, in which all code for the 7-step algorithm is implemented; and second, ROS-enabled wrapper code that utilizes libmoped to read images from the network and to publish the detected objects.

Remove SVN backup 

From CMU
