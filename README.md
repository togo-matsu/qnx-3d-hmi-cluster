# QNX RTOS - Automotive 3D Cluster HMI (RTX 4070)

Real-time automotive digital cluster prototype integrating QNX 8.0 RTOS and Ubuntu 22.04 with GPU Passthrough (NVIDIA RTX 4070).

## Architecture
- **qnx_telemetry/**: 60Hz UDP dynamic telemetry streamer running on QNX 8.0 (Normal / Warning / Emergency Stop).
- **hmi_cluster/**: Hardware-accelerated Qt 6 + QOpenGLWidget HMI with neon gauges and 3D cyber vehicle model.
