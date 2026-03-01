# Indirect Visual Odometry with Optical Flow Project

This repository extends the existing visual-odometry pipeline by replacing descriptor-based matching with an optical-flow (KLT) frontend that tracks keypoints across consecutive frames and stereo image pairs. The main optical-flow implementation lives in `include/visnav/optical_flow.h`, and it is integrated into the system execution in `src/odometry.cpp` (functions `next_step_optical_flow()` and the associated GUI controls). Supporting evaluation and verification are provided via `plot_scripts/` (plotting utilities) and `test/src/test_klt.cpp` (unit tests for the KLT pipeline).

In short:
- **New implementation:** `include/visnav/optical_flow.h` (pyramids, patch extraction, KLT tracking, and related utilities).
- **VO integration:** `src/odometry.cpp` (the `next_step_optical_flow()` pipeline, ID propagation, track-length/lifespan metrics, and visualization).

Evaluation: ATE/RPE metrics and track-length/lifespan comparisons are reported using the tools and scripts provided in this repository (see the evaluation and plotting sections below).

License: the project preserves the BSD 3‑Clause license from the base code and the included third‑party notices. See `LICENSE` and the licenses under `thirdparty/` for details.

# Quick start — run the pipeline
--------------------------------

Build and run the odometry executable from the project root. Examples below use the EuRoC V1_01_easy sequence; change `--dataset-path` to point at any other dataset folder that contains `cam0/data.csv` and `cam0/data/*.png`.

1. Build (from repository root):

```bash
mkdir -p build && cd build
cmake --build build/
```

2. Run with GUI (interactive):

```bash
./odometry --show-gui=true --dataset-path ../data/V1_01_easy/mav0 --cam-calib ../opt_calib.json
```

3. Run non-GUI (process all frames and exit, suitable for batch runs):

```bash
./odometry --show-gui=false --dataset-path ../data/V1_01_easy/mav0 --cam-calib ../opt_calib.json
```

GUI controls (quick reference)
-----------------------------

When running with `--show-gui=true` the left panel exposes checkboxes and buttons to control execution and visualization. Short descriptions of the main controls:

- `show_detected`: toggle drawing of detected keypoints/patch sampling points on the images.
- `show_matches`: toggle visualization of matches between the two views (left/right or frame-to-frame depending on mode).
- `show_inliers`: draw only matches/inliers used by the current PnP/E estimation.
- `show_reprojections`: overlay landmark reprojections and reprojection errors.
- `show_outlier_obs`: show observations flagged as outliers.
- `show_ids`: draw numeric ids next to keypoints/matches.
- `continue_next`: when checked, the program repeatedly calls the `next_step()` processing loop until completion (or until unchecked).
- `continue_next_optical_flow`: when checked, the program repeatedly calls the `next_step_optical_flow()` processing loop until completion (or until unchecked). Note this will disable `continue_next` while active.

Buttons (left panel):

- `next_step` — runs a single iteration of the descriptor-based pipeline implemented in `next_step()`.
- `next_step_optical_flow` — runs a single iteration of the optical-flow pipeline implemented in `next_step_optical_flow()`.
- `save_trajectory` — writes `trajectory_estimated.txt` (TUM format) and triggers saving of track-length and lifespan metrics (see notes under).

Practical tip: use `next_step_optical_flow` when you want to evaluate the optical-flow frontend (it produces the `track_lifespan_*` outputs). Use `next_step` to compare the descriptor-based baseline (`track_length_dm*` files).

Notes and outputs:
- On exit the program writes trajectory and metrics; look for `trajectory_estimated.txt` (TUM format),  `track_length_dm.txt` for Descriptor Matching and `track_lifespan_of.txt` for **`Optical Flow`** files in the working directory.


- You can generate a track-length / lifespan comparison PNG using the helper
script `plot_scripts/plot_compare_lifespans.py`. The script accepts one or
more two-column files (or CSVs with headers) and plots them together.


# From Descriptor Matching to Optical Flow in Visual Odometry

Two pipelines live in `src/odometry.cpp`:

- **`next_step()`**: descriptor-based: detect keypoints and descriptors every frame; match left–right and frame–landmark by descriptor distance and reprojection.
- **`next_step_optical_flow()`**: optical flow–based: track keypoints across frames and left–right with Lucas–Kanade (pyramidal KLT). Correspondences come from the tracker. Landmark identity is propagated via the index map `frame_feat_to_track`.

**What changed in the OF pipeline:**

- **Temporal:** Instead of detecting and matching descriptors on every frame, we track keypoints from the previous frame with pyramidal KLT. An index map (`old_to_new_id_map`) says which current keypoint corresponds to which previous one. Landmark IDs are copied along this map (`frame_feat_to_track`).
- **Stereo:** Instead of descriptor matching between left and right, we track left keypoints into the right image with the same pyramidal KLT. Matches are still filtered with the essential matrix.
- **2D–landmark:** On keyframes, 2D–landmark matches come from the propagated track IDs (no `find_matches_landmarks`). On normal frames, they come directly from `frame_feat_to_track` (no reprojection or descriptor search).
- **Keyframes / normal frames:** Same idea (keyframe = full stereo + map update, normal = pose only), but keyframes use grid-based detection and stereo KLT. Normal frames only run temporal KLT and PnP from propagated matches. A new keyframe is triggered when PnP inliers drop below a threshold.

### Project structure (optical flow)

**Core implementation**
- **`include/visnav/optical_flow.h`** — Pyramidal KLT: `ImagePyramid`, `generateKLTSamplingPoints`, `extractPatchWithStatus`, `extractPatches`, `findBestMatchInNeighborhood`, `trackPoint`, `trackPoints`.

**Where it is used**
- **`src/odometry.cpp`**. The OF pipeline is in `next_step_optical_flow()`: builds pyramids, calls `trackPoints` for temporal (prev→curr) and stereo (left→right), `extractPatches` for visualization, and propagates IDs via `frame_feat_to_track`. Uses `keypoints.h` for `detectKeypoints` (grid-based on keyframes). Still uses `matching_utils.h` for `findInliersEssential` and `map_utils.h` for BA/localization.

**Tests**
- **`test/src/test_klt.cpp`** Unit tests for sampling points, patch extraction, pyramidal tracking, and same/stereo image pairs

**Relevant tree**
```
include/visnav/
  optical_flow.h    # KLT + pyramid + trackPoints / extractPatches
  vo_utils.h        # propagate

src/
  odometry.cpp      # next_step_optical_flow() = OF pipeline
test/
  src/test_klt.cpp  # KLT unit tests
```

---

### Comparison

| Descriptor matching (DM) | Optical flow (OF) |
|---------------------------|--------------------|
| ![DM trajectory](data/images/dm.png) | ![OF trajectory](data/images/of.png) |

We can see here OF has significantly stereo matching and inliers number


### Map comparison on V1_03_difficult

| Descriptor matching (DM) | Optical flow (OF) |
|---------------------------|--------------------|
| ![DM map V1_03](data/images/dm_map_v1_03.png) | ![OF map V1_03](data/images/of_map_v1_03.png) |

OF stays close to the ground truth trajectory, while DM has accumulative drifts making large error.

### Track length comparision

| Track length comparision
|---------------------------|
| ![Traj_com](bm_v1_01_easy/01/compare_lifespans.png) |

OF consistently exhibits higher track lengths, indicating landmarks are mantained over longer temporal spans.

---

**Table:** RMS Absolute Trajectory Error (ATE) and Relative Translation Error (RTE) for DM and OF across EuRoC MAV sequences. Lower is better.

| Sequence       | ATE DM [m] | ATE OF [m] | RTE DM [m] | RTE OF [m] |
|----------------|------------|------------|------------|------------|
| v1_01_easy     | 0.100      | 0.113      | 0.0316     | 0.0283     |
| v1_02_medium   | 6.373      | 0.189      | 0.5014     | 0.0541     |
| v1_03_difficult| 7.809      | 0.206      | 0.4368     | 0.0504     |
| v2_01_easy     | 1.033      | 0.206      | 0.0776     | 0.0377     |
| v2_02_medium   | 3.159      | 0.464      | 0.2547     | 0.0609     |
| mh_01_easy     | 1.539      | 0.646      | 0.0352     | 0.0384     |
| mh_02_easy     | 1.000      | 0.351      | 0.1209     | 0.0427     |
| mh_03_medium   | 5.078      | 0.285      | 0.1351     | 0.0798     |
| mh_04_difficult| 4.733      | 0.590      | 0.1552     | 0.0789     |
| mh_05_difficult| 10.320     | 0.383      | 0.2276     | 0.0785     |
