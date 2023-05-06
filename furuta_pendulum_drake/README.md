# furuta_pendulum_drake

## Setup

Drake installation:
https://drake.mit.edu/apt.html#stable-releases

To have a working executable after installation add to `~/.bashrc`:
`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/drake/lib/`

## Usage

```
ros2 run furuta_pendulum_drake drake_lqr
```