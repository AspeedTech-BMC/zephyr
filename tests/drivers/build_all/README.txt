Overview
--------

Build tests for drivers and sensors on all platforms.

This test might now work for some of the drivers, those need to be addressed in
other tests targeting those special cases.

Tests
-----

drivers:
	build all drivers

sensors_a_h:
	build sensors with name beginning a through h.

sensors_i_z:
	build sensors with name beginning i through z.

sensors_trigger:
	build sensors with trigger option enabled
