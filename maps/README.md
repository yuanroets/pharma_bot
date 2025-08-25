# Maps Directory
================

This directory will store saved maps from SLAM.

## File Types:
- **`.yaml`** - Map metadata files
- **`.pgm`** - Map image files  
- **`.serialized`** - Serialized map files for localization

## Usage:
After running SLAM, save maps here using RViz "Save Map" button or command line:
```bash
ros2 run nav2_map_server map_saver_cli -f /home/ubuntu/dev_ws/src/pharma_bot/maps/my_map
```

## For Localization:
Update `mapper_params_online_async.yaml`:
```yaml
map_file_name: /home/ubuntu/dev_ws/src/pharma_bot/maps/my_map.yaml
mode: localization
```
