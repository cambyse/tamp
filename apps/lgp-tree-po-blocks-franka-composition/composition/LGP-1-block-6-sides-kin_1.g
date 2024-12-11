Include = 'data/franka/franka.ors'
Include = 'composition/table.g'

shape franka_hand (panda_body_8){ type=1 size=[0 0 0 0.005] color=[1 1 0] rel=<T t(0 0 -0.12) d(0 0 0 1)> }

body block_1 { type=9 size=[.1 .1 .1 .01] color=[0.3 0.3 0.3] }	

joint (tableC block_1) { from=<T t(0.0 0 0.07) > type=10 }
 
# rest of the world 
body block_1_ { type=9 size=[.1 .1 .1 .01] color=[0.3 0.3 0.3] }	
body block_2_ { type=9 size=[.1 .1 .1 .01] color=[0.3 0.3 0.3] }

shape id_g(block_1_) { type=9 rel=<T d(-90 0 0 1) t(-0.05 0 0.0)> size=[.02 .1 .1 .01] color=[0 1 0] }
shape id_b(block_2_) { type=9 rel=<T d(-90 0 0 1) t(-0.05 0 0.0)> size=[.02 .1 .1 .01] color=[0 0 1] }

joint (tableR block_1_) { from=<T t(-0.2 -0.15 .07) > type=10 }
joint (tableR block_2_) { from=<T t(0.2 0.0 .07) > type=10 } # has been placed down already
#
 
BELIEF_START_STATE { 
{
 shape id_1(block_1) { type=9 rel=<T t(-0.05 0 0.0)> size=[.02 .1 .1 .01] color=[1 0 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T d(-90 0 0 1) t(-0.05 0 0.0)> size=[.02 .1 .1 .01] color=[1 0 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T d(-180 0 0 1) t(-0.05 0 0.0)> size=[.02 .1 .1 .01] color=[1 0 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T d(-270 0 0 1) t(-0.05 0 0.0)> size=[.02 .1 .1 .01] color=[1 0 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T d(90 0 1 0) t(-0.05 0 0.0)> size=[.02 .1 .1 .01] color=[1 0 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T d(-90 0 1 0) t(-0.05 0 0.0)> size=[.02 .1 .1 .01] color=[1 0 0] }
}
}

