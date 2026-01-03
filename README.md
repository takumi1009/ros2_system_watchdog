# ros2_system_watchdog

[![ROS 2 CI](https://github.com/takumi1009/ros2_system_watchdog/actions/workflows/ros_ci.yml/badge.svg)](https://github.com/YOUR_GITHUB_USERNAME/ros2_system_watchdog/actions/workflows/ros_ci.yml)
![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)

- システム（CPU）の使用率を監視し、ROS 2トピックとして配信するパッケージです。

## Description / 概要
- このパッケージは、実行中のシステムのCPU使用率を `psutil` ライブラリを使用して取得し、ROS 2のトピック `/cpu_usage` に配信します。ロボットシステムの負荷をリアルタイムで監視することを目的としています。

## Requirements / 動作環境
以下の環境で動作確認およびCIテストを行っています。

| 項目 | バージョン / 内容 |
| :--- | :--- |
| **OS** | Ubuntu 22.04 LTS (Jammy Jellyfish) |
| **ROS 2** | Humble Hawksbill |
| **Python** | 3.10 or later |
| **Dependencies** | `python3-psutil` |

## Features / 機能

- **リアルタイム監視**: CPU使用率を `/cpu_usage` トピックへ配信。
- **アラート機能**: 使用率が80%を超えると警告ログを表示。
- **CI連携済み**: GitHub Actionsによりビルドと規約チェックを自動実行。

## Installation / インストール

### Prerequisites / 必要条件
- ROS 2 Humble
- Python 3 `psutil` library

```

Bash
$ sudo apt update
$ sudo apt install python3-psutil

```

## Usage / 使い方

```

Bash

$ ros2 run ros2_system_watchdog watchdog

```

## Node Info / ノード情報
watchdog ノード
システムのCPU負荷を監視し、トピックへ配信します。

配信トピック (Published Topics)

/cpu_usage (std_msgs/Float32): 現在のCPU使用率（0.0 ~ 100.0%）。

ログ出力 (Logging)

CPU使用率が 80% を超えた場合、WARN レベルで警告ログを出力します。

## License / ライセンス
- Copyright (c) 2025 Komiya Takumi
- Licensed under the Apache License, Version 2.0

