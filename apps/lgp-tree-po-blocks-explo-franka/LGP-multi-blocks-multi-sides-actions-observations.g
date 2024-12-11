### Actions
# Check
DecisionRule check {
  X, Y, Z
  { (block X) (side Y) (identified X)! (bottom_facing X Y)! (observed X Y)! (flipped X Z) (hand_empty) (clear X)} # remove bottom_facing ?
  { (in_sight X Y) (observed X Y) (object_put_down X)! komoCheck(X, Y, Z)=1. }
}

# Pick-up, block already identified AND doesn't need flipping
DecisionRule pick-up {
  X, Y, Z, T
  { (block X) (location Y) (side T) (on_table X Y) (flipped X Z) (colored X T) (identified X) (clear X) (needs_flipping X)! (hand_empty) }
  { (on_table X Y)! (holding X) (hand_empty)! (object_put_down X)! (clear X)! komoPickUp(X Y Z T)=1. }
}

# Pick-up, block needs fleeping (might be identified or not)
DecisionRule pick-up { # Pick up of a block needing flipping. can be dead-end if picked up without identification!
  X, Y, Z, T
  { (block X) (location Y) (side T) (on_table X Y) (flipped X Z) (colored X T) (clear X) (needs_flipping X) (hand_empty) }
  { (on_table X Y)! (holding X) (hand_empty)! (object_put_down X)! (clear X)! komoPickUp(X Y Z T)=1. }
}

# Put-down, no flipping
DecisionRule put-down { 
  X, Y, Z
  { (block X) (location Y) (side Z) (holding X) (identified X) (needs_flipping X)! (colored X Z) }
  { (holding X)! (hand_empty) (on_table X Y) (orientation_correct X) (object_put_down X) (clear X) komoPutDown(X Y Z)=1. }
}

# Put-down-flipped
DecisionRule put-down-flipped {
  X, Y, Z
  { (block X) (location Y) (side Z) (holding X) (flipped X FALSE) (needs_flipping X) (colored X Z)}
  { (holding X)! (hand_empty) (bottom_facing X side_5)! (bottom_facing X side_2) (flipped X TRUE) (flipped X FALSE)! (needs_flipping X)! (orientation_correct X) (object_put_down X) (clear X) (on_table X Y) komoPutDown(X Y Z)=1. }
}

# Stack, no flipping
DecisionRule stack { 
  X, Y, Z
  { (block X) (block Y) (side Z) (holding X) (clear Y) (identified X) (needs_flipping X)! (colored X Z) }
  { (holding X)! (hand_empty) (clear X) (clear Y)! (orientation_correct X) (object_put_down X) (on X Y) komoStack(X Y Z)=1. }
}

# Stack-flipped
DecisionRule stack-flipped {
  X, Y, Z
  { (block X) (block Y) (side Z) (holding X) (clear Y) (flipped X FALSE) (needs_flipping X) (colored X Z)}
  { (holding X)! (hand_empty) (clear X) (clear Y)! (bottom_facing X side_5)! (bottom_facing X side_2) (flipped X TRUE) (flipped X FALSE)! (needs_flipping X)! (orientation_correct X) (object_put_down X) (clear X) (on X Y) komoStack(X Y Z)=1. }
}

# Unstack - assumes object already has the right orientation
#DecisionRule unstack {
#  X, Y
#  { (block X) (block Y) (clear X) (on X Y) (hand_empty) }
#  { (on X Y)! (holding X) (hand_empty)! (clear X)! (clear Y) komoUnStack(X Y)=1. }
#}

DecisionRule unstack { # Pick up of a block already identified and needed no flipping
  X, Y, Z, T
  { (block X) (block Y) (side T) (on X Y) (flipped X Z) (colored X T) (identified X) (clear X) (hand_empty) }
  { (on X Y)! (holding X) (hand_empty)! (object_put_down X)! (clear X)! (clear Y) komoUnStack(X Y Z T)=1. }
}

### Observation Model
# Observation model (side identification) of an unknown side
Rule {
  X, Y, Z
  { (block X) (side Y) (id Z) (NOT_OBSERVABLE colored X Y) (in_sight X Y) (NOT_OBSERVABLE is X Z)}
  { (in_sight X Y)! (colored X Y) (NOT_OBSERVABLE colored X Y)! (is X Z) (NOT_OBSERVABLE is X Z)! (identified X)}
}

# we deduced already which side is colored
Rule {
  X, Y, Z
  { (block X) (side Y) (id Z) (colored X Y) (in_sight X Y) (NOT_OBSERVABLE is X Z)}
  { (in_sight X Y)! (is X Z) (NOT_OBSERVABLE is X Z)! (identified X)}
}

# Observation model (block identification)
#Rule {
#  X, Y
#  { (block X) (id Y) (NOT_OBSERVABLE is X Y) (in_sight X) }
#  { (in_sight X)! (is X Y) (identified X)  (NOT_OBSERVABLE is X Y)!}
#}

# Other Rules
#Apply identification to the ON
Rule {
  X, Y, Z, T
  { (block X) (block Y) (id Z) (id T) (is X Z) (is Y T) (on X Y) }
  { (on Z T)}
}

Rule {
  X, Y, Z, T
  { (block X) (block Y) (id Z) (id T) (is X Z) (is Y T) (on X Y)! (on Z T)}
  { (on Z T)!}
}

Rule { # remove in sight
  X, Y
  { (block X) (side Y) (colored X Y)! (in_sight X Y) }
  { (in_sight X Y)!}
}

Rule { # remove in sight
  X, Y
  { (block X) (side Y) (NOT_OBSERVABLE colored X Y)! (in_sight X Y) }
  { (in_sight X Y)!}
}

# Helpers
Rule {
  X,
  { (block X) (colored X side_1) }
  { (orientation_correct X)}
}

Rule {
  X,
  { (block X) (colored X side_4) (flipped X FALSE) }
  { (needs_flipping X)}
}

Rule {
  X,
  { (block X) (colored X side_5) (flipped X FALSE) }
  { (needs_flipping X)}
}

# last side deduction
Rule {
  X,
  { (block X) (observed X side_0) (observed X side_1) (observed X side_2) (observed X side_3) (observed X side_4) (identified X)! }
  { (NOT_OBSERVABLE colored X side_5)! (colored X side_5)}
}

Rule {
  X,
  { (block X) (observed X side_0) (observed X side_1) (observed X side_2) (observed X side_3) (observed X side_5) (identified X)! }
  { (NOT_OBSERVABLE X colored side_4)! (colored X side_4)}
}

Rule {
  X,
  { (block X) (observed X side_0) (observed X side_1) (observed X side_2) (observed X side_4) (observed X side_5) (identified X)! }
  { (NOT_OBSERVABLE X colored side_3)! (colored X side_3)}
}

Rule {
  X,
  { (block X) (observed X side_0) (observed X side_1) (observed X side_3) (observed X side_4) (observed X side_5) (identified X)! }
  { (NOT_OBSERVABLE colored X side_2)! (colored X side_2)}
}

Rule {
  X,
  { (observed X side_0) (observed X side_2) (observed X side_3) (observed X side_4) (observed X side_5) (identified X)! }
  { (NOT_OBSERVABLE colored X side_1)! (colored X side_1)}
}

Rule {
  X,
  { (block X) (observed X side_1) (observed X side_2) (observed X side_3) (observed X side_4) (observed X side_5) (identified X)! }
  { (NOT_OBSERVABLE colored X side_0)! (colored X side_0)}
}
