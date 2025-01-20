# Installation

```sh
# In this folder
git submodule update --init
# In your ROS2 workspace
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

# API

See the documentation for the ROS API.

# Test code style
To check the C++ and Python code style run:
```sh
pip install black
black --check **/**.py
# Fix by using
black **/**.py

sudo apt install clang-format # preferably version 14, 10 should be fine
clang-format --dry-run --Werror ./**/**.cpp -style=llvm
# Fix by using
clang-format --Werror ./**/**.cpp -style=llvm -i
```
