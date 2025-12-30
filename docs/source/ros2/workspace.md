# ROS2 Workspace

## Related Pages
- colcon
- rosdep

## Definisi

Workspace adalah direktori yang di dalamnya terdapat *package* ROS2.

## Membuat Workspace Baru

Misal namanya adalah ```ros2_ws```.

```bash
mkdir -p ~/ros2_ws/src
```

Direktori ```ros2_ws``` sebagai workspace yang di dalamnya terdapat direktori ```src```.

Direktori ```ros2_ws/src``` adalah tempat dimana *package* ROS2 diletakkan.

## Build

Di ROS2, build dilakukan oleh ```colcon```.

Build harus dilakukan di dalam direktori ```ros2_ws``` jangan di dalam ```ros2_ws/src```.

Setelah build, akan muncul 3 direktori baru, yaitu:
- ```build```
- ```install```
- ```log```

**build** adalah direktori tempat proses build terjadi. Jika terjadi error ketika build dan sudah diperbaiki tetapi masih error, coba untuk menghapus direktori ini.

**install** adalah direktori paling penting dimana executable, library, dan package share diletakkan.

**log** adalah direktori history build yang telah dilakukan.

## Note!

- ```source install/setup.bash``` sebenarnya mendaftarkan package yang ada di dalam folder install agar terdeteksi oleh sistem ROS2