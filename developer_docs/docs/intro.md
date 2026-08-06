---
sidebar_position: 1
title: Overview
description: Detailed developer roadmap for the LiDAR Evaluation Framework.
---

# 🚀 LiDAR Evaluation Framework

## Overview

Modern robotics perception pipelines are often constrained by the physical limitations and variability of LiDAR sensors. This framework provides a structured, reproducible way to evaluate and benchmark LiDAR performance across configurable environments.

At its core, the system models environments as **zone-based evaluation spaces**, defined through YAML configuration files. These zones allow users to precisely specify regions of interest, apply spatial filtering to incoming point clouds, and establish consistent evaluation baselines for repeatable sensor comparison.

---

## 🧩 Core Components

### 1. **Dynamic Zone Engine (YAML → Spatial Regions)**
A plugin-driven system that turns declarative zone definitions into resolved spatial regions, using the scene's TF frames as ground truth. Those regions are what every metric is scored against, so evaluation stays constrained to the areas that matter and stays consistent across runs and sensors.

Each zone **geometry** is a self-contained plugin selected by registry. A plugin is the single place defining everything about its geometry — how it is parsed, how point clouds are filtered against it (both as a 3D region and as an angular frustum from the sensor's viewpoint), and how it is drawn for visualization. Planar and cylindrical geometries ship today; adding another means writing one plugin and registering it, with no changes to the engine or any consumer.

At runtime the engine resolves zone poses from TF and publishes the resulting baseline profiles to the rest of the pipeline, along with zone overlays for RViz.

---

### 2. **Plugin-Based LiDAR Metrics Library**
The analytical core — a pure-Python plugin engine with no ROS dependency, so metrics can be developed and tested in isolation from the runtime. Each metric is a plugin declared in a registry, enabled and tuned entirely through configuration; roughly two dozen ship today.

Because the bench observes a *known static scene*, the correct answer is derivable from the scene's geometry and a perfect sensor would return it with zero variance. Every metric is some form of deviation — from ground-truth geometry, from the sensor's advertised specification, or from its own previous scan.

A metric declares whether it needs a zone as a **3D region** (points inside a volume) or as an **angular frustum** (the rays the sensor aimed at it) — measuring what the sensor *should* have hit requires knowing where it aimed, not just what came back. The engine supplies the right view and routes each metric to the zone geometries it was written for, so metrics never filter their own points.

Metrics accumulate across scans and reduce once the run ends, which allows whole-run measurements no single frame can express: cells that never return across an entire recording, scan-to-scan stability on a stationary scene, or interlaced scanners that need many sweeps to complete one pattern.

Coverage spans dropout and density, geometric accuracy against known surfaces, scan-to-scan repeatability, local structure and noise, and intensity response versus expected material reflectance.

---


### 3. **Reporting Pipeline Node**
A dedicated processing node that aggregates evaluation outputs into structured reports. It:
- Computes final metrics
- Organizes run metadata
- Exports reproducible summaries

These outputs support downstream analysis, logging, and CI-style benchmarking workflows.

---


### 4. **Results Storage Backend Library**
A swappable persistence layer that decouples *where results live* from the code producing and consuming them.

A single interface defines both halves of the contract — writing a completed run (metrics, visualization data, recorded bags) and reading it back for analysis. One backend implements the whole contract, and each consumer uses the half it needs: the reporting pipeline writes, the dashboard reads.

Backends are selected by registry, not by import. A Google backend (Sheets + Drive) ships today; adding another means implementing the interface and enabling it in the registry, with no changes to either consumer.

Authentication is independently pluggable on the same principle, so how credentials are obtained can change without touching the storage backend.

---

### 5. **Visualization Dashboard**
An interactive dashboard for exploring LiDAR evaluation results. It supports:
- Point cloud visualization
- Zone overlays
- Metric breakdowns
- Side-by-side sensor comparisons

This makes performance analysis intuitive and visually grounded.

---

### 6. **Configuration & Setup CLI tool + just recipes**
A run is defined by exactly two files: an **environment** config (the physical scene — zones, obstacles, geometry) and a **LiDAR** config (sensor identity, mount pose, driver, sweep definition). The `polysetup` CLI reads that pair and generates everything downstream — the bench URDF and the config for every node in the pipeline — so no node config is edited by hand.

Configuration errors surface at setup time with a clear message rather than deep inside a run. A zone that exceeds the sensor's field of view, for instance, fails before anything launches.

A `Justfile` wraps the CLI so a full run is a handful of recipes:

| Recipe | Purpose |
|---|---|
| `just setup-ws <environment> <lidar>` | Generate configs for this environment/LiDAR pair, then build |
| `just launch-bench` | Bring up the pipeline |
| `just start-run` / `just stop-run` | Start or stop an evaluation |
| `just enable-bag-recording` / `just disable-bag-recording` | Record fresh bags, or evaluate existing ones |

Configs are named, not pathed — `just setup-ws conference_room hummingbird` resolves each name against the config tree, and refuses to guess: an unknown name lists what is available, an ambiguous one lists the candidates.

The recording toggle switches the pipeline between its two modes. With recording **enabled**, the automation manager owns the run end to end — driver lifecycle, angle sweeps, parameter sweeps, and bag capture per case. With it **disabled**, the recording chain is left out of the launch and the evaluation replays already-recorded bags, which is the fast path for iterating on metrics without a sensor attached.


---

## 📦 Getting Started

This section covers how to pull the repository, orchestrate your containerized environment, launch the runtime engine, and execute core operations.

### Cloning the Repository

First, clone the repository and navigate into the project root:

```bash
git clone https://github.com/polymathrobotics/lidar_eval_framework
cd lidar-eval-framework
