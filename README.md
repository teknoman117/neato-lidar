# neato-lidar

Renders scans from neato lidar (attached to /dev/ttyACM0)

## Dependencies

OpenCV

## Building

```
git clone https://github.com/teknoman117/neato-lidar
cd neato-lidar
cmake -GNinja -DCMAKE_BUILD_TYPE=Release -B build .
cmake --build build -j
build/neato-lidar
```