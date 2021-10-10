
## dependencies
- tinyxml2 xml解析器

```bash
git clone https://github.com/leethomason/tinyxml2.git
cd tinyxml2 && sudo make install
```
## coordination
- 大地坐标系 xyz(东-北-天)
- 车体坐标系 xyz(前-左-上)
- GPS 坐标系 xyz(右-前-上)
- 禾赛雷达姿标系 xyz(左-后-上)
- 德尔福毫米波雷达 xy(右-前)

## memorandum
- 可将rtk封装到ROS包，便于监控节点运行状态
