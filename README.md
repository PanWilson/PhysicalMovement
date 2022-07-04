# PhysicalMovement
**Physical movement component** with base functionality like:
- max speed and acceleration
- custom gravity scale
- 2 stages jump (set height, distance and jump velocity)
- physical interaction with base o the pawn and relative velocity
- basic AI implementation

## Future plans
- gravity direction
- option to switch facing mode (velocity/camera)
- swap line trace for sphere trace for floating
- options for pawn orientation (constrain XYZ)
- asyn physics tick(blocked by current state of chaos in UE5)
- network replication(blocked by current state of chaos in UE5)
- pawn stacking

## Technical setup

Tested on UE5.0

![](https://thumbs.gfycat.com/GiftedCloudyKitten-size_restricted.gif)

better quality https://imgur.com/a/QoehIjj

## Issues
- UE5 has a bug where character jitters, as temp solution use p.Chaos.Solver.SleepEnabled 0 command
