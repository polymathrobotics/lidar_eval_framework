---
sidebar_position: 2
title: Architecture / Core Concepts
toc_min_heading_level: 2
toc_max_heading_level: 4
---

import ArchitectureDiagram from '@site/src/components/ArchitectureDiagram';

# 🏗️ Core System Design

The system is composed of 9 ROS2 packages and 3 supporting Python packages that provide utility tooling, configuration setup interfaces, and visualization dashboards. Together, these components form a modular pipeline for LiDAR evaluation, where data flows through clearly separated stages of ingestion, processing, spatial filtering, visualization, and reporting.

The architecture diagram below illustrates the overall system design and how data moves across each submodule, from raw sensor input through to final evaluation outputs.

---

## 🧠 System Architecture Diagram

<ArchitectureDiagram />

---


# 📦 Repository Package Structure

The framework is organized into modular ROS 2 packages, each responsible for a specific stage of the LiDAR evaluation pipeline. This separation allows configuration management, sensor processing, evaluation, reporting, and hardware integration to evolve independently while maintaining a consistent data flow across the system.

---

## ROS 2 Core Packages

### **`lidar_bench_bringup`**

The primary entry point for launching the LiDAR evaluation bench. This package initializes the required ROS 2 nodes, loads global runtime parameters, and handles the launch coordination for the complete evaluation pipeline.

---

### **`lidar_bench_interfaces`**

Defines the custom ROS 2 interfaces used throughout the framework. This package contains the custom topics, services, and action definitions required for communication between the orchestrator, filtering nodes, metrics backend, and reporting components.

---

### **`lidar_bench_orchestrator`**

The central orchestration package responsible for managing evaluation runs. It acts as the state machine for the bench, coordinating the lifecycle of data playback, point cloud processing, metric collection, and reporting workflows.

---

### **`lidar_metrics_library`**

A modular, plugin-based math and analysis library that computes sensor performance metrics (such as range error and depth accuracy) from filtered point cloud populations. It exposes an extensible API, allowing developers to implement custom metrics tailored to specific evaluation criteria.

---

### **`lidar_zones_generator`**

Generates the spatial evaluation environment used during runtime. This package consumes static configuration files (such as YAML) containing physical zone definitions, constructs the geometric bounds, and publishes the corresponding URDF models and spatial transforms (TF) to the ROS 2 system.

---

### **`lidar_automation_manager`**

Provides automation utilities for high-level evaluation workflows, including automated rosbag recording, hardware rigging controls, and batch experiment management to ensure repeatable testing scenarios.

---

### **`lidar_reporting`**

Provides the reporting pipeline for aggregating, storing, and visualizing evaluation results. This package collects computed metrics from the library, processes pooled statistics, writes to structured databases, and generates finalized performance reports.

---

### **`lidar_pointcloud_filter`**

The primary data-path processing node responsible for isolating sensor data. It consumes incoming raw point clouds (`sensor_msgs/msg/PointCloud2`) and uses the geometric definitions from the zones generator to apply high-frequency spatial, projective, and boundary-box filtering.

---

### **`arduino_eth_bridge`**

Provides hardware communication support for external motor and actuator controllers. This package enables integration with firmware-controlled hardware components used for automated sensor positioning and motion control.

---

# 📁 Configuration Folders

## **`environment_configs`**

Contains environment definitions used for LiDAR evaluation. Each configuration describes the geometric structure of an evaluation environment, including zones, obstacles, reference surfaces, and sensor test scenarios.

---

## **`lidar_configs`**

Contains LiDAR-specific configuration files describing sensor properties, mounting parameters, TF relationships, topics, and evaluation settings required to integrate a sensor into the framework.

---

# 🐍 Python Support Modules

## **`polysetup`**

A graphical setup utility that simplifies evaluation configuration. Users can load and validate LiDAR configurations, environment definitions, and evaluation parameters before launching experiments.

---

## **`polyview_app`**

A visualization dashboard for inspecting LiDAR evaluation results. It provides interactive visualization of point clouds, evaluation zones, metric outputs, and sensor comparison results.

---

## 🔄 How Everything Works Together

### 1. Configuration Loading
The system begins by loading structured YAML configuration files defining:
- LiDAR sensor setup
- Environment zones
- Evaluation parameters

These are parsed into a unified runtime configuration model.

---

### 2. System Initialization
A runtime fixture builds the evaluation environment, including:
- URDF construction of the test bench
- TF tree setup for sensor alignment
- ROS2 launch initialization

---

### 3. Processing Loop
Once running, the system executes a continuous evaluation pipeline:

- Raw point cloud data is ingested (live sensor or rosbag)
- Data is transformed into a common reference frame
- Zone-based filtering is applied
- Filtered point clouds are passed into metric plugins
- Results are aggregated per zone and per sensor

---

### 4. Evaluation & Reporting
Metrics are computed using a plugin-based framework and then:
- Stored as structured YAML/JSON reports
- Sent to visualization dashboards
- Optionally exported to external systems (Grafana, Sheets, etc.)

---

## 🧩 Core Concepts

### 📍 Zone-Based Evaluation Model
The environment is divided into configurable spatial zones defined via YAML. These zones enable:
- Region-specific evaluation
- Noise filtering outside areas of interest
- Consistent benchmarking across sensors

---

### 🔌 Plugin-Based Metrics System
Metrics are not hardcoded. Instead, they are:
- Implemented as plugins
- Dynamically loaded at runtime
- Applied per-zone to filtered point clouds

This allows domain-specific evaluation without modifying core infrastructure.

---

### 🔁 Reproducible Evaluation Pipeline
Every evaluation run is fully reproducible through:
- YAML-based configuration
- Fixed TF tree definitions
- Deterministic processing pipeline

---

### 📊 Multi-Layer Output System
Evaluation results are surfaced through multiple channels:
- Structured report files (YAML/JSON)
- Visualization dashboards
- External analytics tools (Grafana, Sheets, etc.)

---

## 🧭 Summary

This architecture enables a fully modular LiDAR evaluation pipeline where:
- Configuration is externalized (YAML-driven)
- Zones are extensible for differet objects and shapes as long as the boundaries are defined correctly
- Processing is deterministic and reproducible
- Metrics are extensible via plugins
- Visualization and reporting are decoupled from core computation
- database backend are configurable via plugins


Together, these components form a scalable benchmarking framework for evaluating LiDAR performance across diverse robotics environments.
