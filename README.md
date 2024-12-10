# Parallel Optimization for Robotics (POfR)

## Project Overview

This project focuses on optimizing **trajectory planning for robotic systems** using the Rapidly-Exploring Random Trees (RRT) algorithm. RRT is widely used in robotics for path planning, but its computational expense—especially in high-dimensional spaces—limits its real-time applicability. By parallelizing key components of the algorithm and integrating cache optimization techniques, this project aims to significantly reduce computation time, making RRT more practical for real-world applications such as search-and-rescue missions.

## Motivation

Trajectory planning in robotics often involves solving computationally intensive tasks like inverse kinematics and collision checking. These tasks become bottlenecks when operating in high-dimensional spaces, where robots have multiple degrees of freedom. Optimizing these processes is critical for time-sensitive scenarios, such as:

- **Search and rescue operations**: Robots need to navigate unstable environments quickly to save lives.
- **Disaster response**: Planning safe trajectories in debris-filled areas is essential for delivering medical supplies or maneuvering objects.

By leveraging GPU-based parallelization and cache-aware optimizations, this project seeks to enhance the speed and efficiency of RRT, enabling faster trajectory planning under such critical conditions.

## Key Objectives

1. **Parallelizing RRT**:
   - Redesign the RRT algorithm to utilize GPU parallelism.
   - Explore different parallelization strategies:
     - Global shared tree (all threads grow a single tree with memory safety mechanisms).
     - Independent trees per thread (selecting the best path from multiple results).

2. **Optimizing Inverse Kinematics**:
   - Apply iterative solvers and factorization techniques to solve linear systems efficiently.
   - Introduce cache-friendly data organization (e.g., row-major order) and cache tiling to minimize memory latency.

3. **Enhancing Collision Checking**:
   - Implement both serial and parallelized sample validation methods.
   - Use CUDA-based parallelism to validate trajectory segments efficiently.

4. **Analysis of Algorithm**
   - where is most of the algorithms time spent?
   - what is most privvy to optimization?
  


## Methodology

### Parallelization Techniques
- **Sampling**: Threads will perform random sampling in parallel, either contributing to a shared tree or maintaining independent trees.
- **Collision Checking**: Validate trajectory segments concurrently using GPU threads.

### Comparative Analysis
- Develop both serial and parallel implementations of RRT.
- Compare performance metrics (e.g., runtime, success rate) between CPU-based serial methods and GPU-accelerated parallel methods.

### Tools & Resources
- Programming languages: Python (with CUDA extensions)
- Libraries: CUDA for GPU acceleration.
- Reference papers on massively parallelized RRT and cache optimization techniques.

## Team Contributions

| Team Member       | Primary Task                          | Secondary Task                         |
|--------------------|---------------------------------------|----------------------------------------|
| Mahdi             | IK Multithreading Optimization        | Resampling & Shortcutting              |
| Mario             | IK LU Method                          | Serial Collision Checking              |
| Moises            | Data collection/Testing/Algo Analysis | Parallel Sampling (Global Tree)        |
| Ines              | Parallel Sampling (Independent Trees) | Serial Sampling Implementation         |
| Faustina          | Parallel Collision Checking           | Resampling & Shortcutting              |

## Future Work

Potential extensions of this project include:
- Parallelizing other trajectory planning algorithms like A* or Probabilistic Roadmaps or RRT*
- Exploring multi-agent systems where multiple robots collaborate on mapping or navigation tasks.

## Note
  -Initial boilerplate code from Columbia's MECS 4603 (Applied Robotics) as taught by Matei Ciocarlie in Fall 2024, code in src/assignment3/assignment3/mp.py is original for the sake of this project
