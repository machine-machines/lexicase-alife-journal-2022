# libXeres
Extensible Evolutionary Robotics Environment Simulator

#### Current tasks
* ```WallTaskWorld``` 
    * Walk to target across the wall
    * Environments: 100x different heights of wall
* ```TurnTaskWorld```
    * Turn and head to target
    * Environments: 100x different angles to turn to
* ```FloorIsLavaTaskWorld```
    * "floor is lava" - maximise time in the air
    * Environments: 100x different gravities
* ```PreciousCargoTaskWorld```
    * A box on the top, maximise time carrying it
    * Can go in any direction, or stand still (no distance metric)
    * Environments: 100x different box masses
    * Box is locked to the top with a joint for the first second


