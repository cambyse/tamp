# Check
DecisionRule check {
  X
  { (block X) (holding X)! (identified X)! (hand_empty) }
  { (in_sight X) komoCheck(X)=1. }
}

# Pick-up
DecisionRule pick-up {
  X, Y
  { (block X) (location Y) (clear X) (on_table X Y) (hand_empty) }
  { (on_table X Y)! (holding X) (hand_empty)! (clear X)! (clear Y) komoPickUp(X Y)=1. }
}

# Put-down
DecisionRule put-down {
  X, Y
  { (block X) (location Y) (holding X) }
  { (holding X)! (hand_empty) (clear X) (on_table X Y) komoPutDown(X Y)=1. }
}

# Stack
DecisionRule stack {
  X, Y
  { (block X) (block Y) (holding X) (clear Y) }
  { (holding X)! (hand_empty) (clear X) (clear Y)! (on X Y) komoStack(X Y)=1. }
}

# Unstack
DecisionRule unstack {
  X, Y
  { (block X) (block Y) (clear X) (on X Y) (hand_empty) }
  { (on X Y)! (holding X) (hand_empty)! (clear X)! (clear Y) komoUnStack(X Y)=1. }
}

### Rules / Observation Model
#Observation model
Rule {
  X, Y
  { (block X) (id Y) (NOT_OBSERVABLE is X Y) (in_sight X) }
  { (in_sight X)! (is X Y) (identified X)  (NOT_OBSERVABLE is X Y)!}
}

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
