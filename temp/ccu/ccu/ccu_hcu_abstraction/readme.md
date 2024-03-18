# Package ccu_hcu_abstraction

**run:**

> ```bash
> ros2 run ccu_hcu_abstraction main
> ```

**launch:**

> ```bash
> ros2 launch ccu_hcu_abstraction normal.launch.py
> ros2 launch ccu_hcu_abstraction debug.launch.py
>```

**test:**

see [../test/test-suite.md](../test/)

\--- **open points** ---

```python
# in python code is
import tf2_ros
```

```bash
# so i guess (?) it needs (done by rosdep)
sudo apt install ros-galactic-tf-transformations
sudo apt install  ros-humble-tf-transformations

# ! Question !
what to enter in:
        package.xml  requirements.txt  setup.py
```

https://github.com/ros/rosdistro/blob/master/galactic/distribution.yaml
