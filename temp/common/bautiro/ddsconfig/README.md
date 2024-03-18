# Cylcone DDS Configuration file ...
... for ROS2 Nodes running on different machines

1. adapt this file with current IP adress in `<PEERS>`

2. This `ENV` variable must be set with path to the file (Hint: put this in your `~/.bashrc`)

```
export CYCLONEDDS_URI=file://$PWD/cyclonedds_bautiro.xml
```

3. see in our Wiki:  
   [inside-docupedia.bosch.com//bautiro/Communication between ROS2 Computer](https://inside-docupedia.bosch.com/confluence/display/bautiro/Communication+between+ROS2+Computer)


