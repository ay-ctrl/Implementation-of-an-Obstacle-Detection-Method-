# Implementation-of-an-Obstacle-Detection-Method

## Project Overview

This project explores a hybrid obstacle detection pipeline for planetary rover navigation by combining stereo vision and image processing techniques in a simulated environment. The implementation is inspired by and adapted from a research paper on vision-based obstacle detection for rover stereo images.

Stereo image pairs are captured in Unity and processed offline to estimate depth and detect terrain obstacles. The primary goal is to experiment with 2D–3D data fusion, understand the working principles of the referenced method, and study how perception pipelines can be structured for rover-like systems.

This implementation is a simplified, educational adaptation of the original method rather than a fully optimized production-ready system.

---

## Repository Structure

### `StereoCaptureSaver.cs`

Unity script responsible for capturing and saving left–right stereo image pairs from the simulation environment.
It is used to generate the dataset required for depth reconstruction and obstacle analysis.

### `digAPI.py`

Contains the main image processing pipeline.

This module:

* Computes disparity maps from stereo image pairs
* Reconstructs 3D point clouds
* Performs grayscale segmentation
* Evaluates terrain roughness and elevation
* Generates the final obstacle map

It represents the core perception logic of the project.

### `DIGITAL IMAGE PROCESSING.pdf`

Full project report explaining:

* Theoretical background
* Methodology
* Enviroment and project details
* Algorithmic design choices
* Experimental evaluation

---

## Processing Pipeline

1. Capture stereo image pairs in Unity
2. Compute disparity using dense stereo matching
3. Reconstruct 3D point cloud
4. Perform 2D grayscale segmentation
5. Fuse 2D and 3D information
6. Generate obstacle detection map

---

## Future Work

Potential improvements include:

* Testing on real planetary stereo datasets with more images
* Increasing robustness to illumination changes and noise
* Replacing rule-based thresholds with learning-based methods
* Optimizing the pipeline for real-time performance
* Integrating into a full robotic navigation framework
