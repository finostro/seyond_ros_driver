# 07_how_to_fuse_multiple_lidars

## 7.1 Related Parameters

**fusion_enable**: enable fusion

**fusion_topic**: topic for fusion publish

Note: Using this function requires ensuring that all LiDARs are time-synchronized.

## 7.2 config.yaml

Sample Description: Connect two LiDARs and enable fusion_enable.

```yaml
common:
  log_level: info                                     #Log level: info, warn, error
  fusion_enable: true                                 #Multi-lidar fusion enable
  fusion_topic: /iv_points_fusion                     #Multi-lidar fusion topic
lidars:
  - lidar:
      frame_topic: /iv_points                         #Frame topic

      lidar_name: seyond                              #Lidar name
      lidar_ip: 172.168.1.10                          #Lidar ip
      port: 8010                                      #Lidar port
      udp_port: 8010                                  #Udp port, if < 0, use tcp
      
  - lidar:
      frame_topic: /iv_points1                        #Frame topic

      lidar_name: seyond1                             #Lidar name
      lidar_ip: 172.168.1.11                          #Lidar ip
      port: 8011                                      #Lidar port
      udp_port: 8011                                  #Udp port, if < 0, use tcp

      max_range: 2000.0                               #Max range
      min_range: 0.4                                  #Min range
      coordinate_mode: 3                              #Coordinate mode, x/y/z, 0:up/right/forward 3:forward/left/up
```
