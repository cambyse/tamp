Include = 'data/franka/franka.ors'

shape franka_hand (panda_body_8){ type=1 size=[0 0 0 0.005] color=[1 1 0] rel=<T t(0 0 -0.12) d(0 0 0 1)> }

body tableR{ type=9, X=<T t(0.0 -0.55 0.1)>, size=[0.6 0.6 .04 .01], color=[.2 .2 .7] }

body block_1 { type=9 size=[.1 .1 .1 .01] color=[0.3 0.3 0.3] }	
body block_2 { type=9 size=[.1 .1 .1 .01] color=[0.3 0.3 0.3] }	
body block_3 { type=9 size=[.1 .1 .1 .01] color=[0.3 0.3 0.3] }	
body block_4 { type=9 size=[.1 .1 .1 .01] color=[0.3 0.3 0.3] }	

joint (tableR block_1) { from=<T t(.15 -0.15 .07) > type=10 } # from=<T .1 .3 .07 0.707107 0 0 0.707107 >
joint (tableR block_2) { from=<T t(-.15 -0.15 .07) > type=10 } # from=<T -.1 .3 .07 0.707107 0 0 0.707107 >
joint (block_1 block_3) { from=<T t( 0.0 0.0 .1 ) > to=<T > type=10 } 
joint (block_2 block_4) { from=<T t( 0.0 0.0 .1 ) > to=<T > type=10 } 


BELIEF_START_STATE { 
{
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}

{
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}


{
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}

{
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
{
 shape id_1(block_1) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 1] }
 shape id_3(block_3) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_2(block_2) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[0 1 0] }
 shape id_4(block_4) { type=9 rel=<T t(0.0 0.05 0.0)> size=[.1 .02 .1 .01] color=[1 0 0] }
}

}
