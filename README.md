# PhysicalMovement
**Physical movement component** with base functionality like:
- max speed and acceleration
- custom gravity scale
- 2 stages jump(set height, distance and jump velocity)
- physical interaction with base o the pawn and relative velocity

## Future plans
- gravity direction
- option to switch facing mode(velocity/camera)
- swap line trace for sphere trace for floating
- options for pawn orientation(constrain XYZ)
- network replication(posibbly with network prediction plugin)
- pawn stacking

## Technical setup
Tested on UE5.0

![](https://thumbs.gfycat.com/GiftedCloudyKitten-size_restricted.gif)

better quality https://imgur.com/a/QoehIjj

## Issues
- UE5 has a bug where character jitters, as temp solution use p.Chaos.Solver.SleepEnabled 0 command
