# Cyan FTC

A simple motion planning library made for First Tech Challenge. 


## Welcome!

Cyan is a passion project **independent** from any FTC team. Its goal is to make advanced automated movement more accessible and more efficient for everyone. There may be a diverse selection of libraries, however this one is targeted at rookie FTC teams just starting out with programming. 

Any contributions are welcome, just make sure to follow the contribution rules listed at: **Coming Soon**. 

P.S. If you are using this in the Greek FTC Championship, please find me to discuss about your development experience. I am more than happy to discuss any issues.

## Installation

To begin, there are some necessary components for the use of this library.
- Java development kit (JDK)
- An android SDK compatible java IDE

If you do not already have one, please download a copy of the First Tech Challenge SDK which can done from [here](https://github.com/FIRST-Tech-Challenge/FtcRobotController).

1. Open the project on your preferred IDE / text editor and locate the `build.dependencies.gradle` file.

2. Locate the `repositories` section and add:
```
mavenCentral()
```

3. Locate in the `TeamCode` module the file `build.gradle` and add:
```
dependencies {
    implementation: "io.github.com.github.bouyio:CyanFTC:1.0.0"
}
```

4. Sync the gradle changes and build.

## Getting Started

Developing with this library requires some basic knowledge of java and the FTC SDK.
This is a high level overview of the capabilities of the library.
For analytic package documentation, check out the projects javadoc.

### Localization 

For every other system to function the robot needs to have a compatible localization system.
Any localization system can be made compatible by implementing the `PositionProvider` interface.

However, for ease of use one may use the pre-made localization systems.

If you are planning to use the IMU for localization, you can use the `GyroTankOdometry` class. Its use may look like this.

Initialization:
```
// x represents the intial position of the bot in the x axis.
// y represents the intial position of the bot in the y axis.
// theta represents the initial heading of the bot.

GyroTankOdometry odometry = new GyroTankOdometry(x, y, theta);
```

Updating:
```
// left represents the input of the left encoder.
// right represents the input of the right encoder.
// angle represents the input of the IMU.

odometry.updateMeasurements(left, right, angle);
```

If you are not planning to use the IMU for localization, you can use the `TankKinematics` class. Its use may look like this.

Initialization:
```
// x represents the intial position of the bot in the x axis.
// y represents the intial position of the bot in the y axis.
// theta represents the initial heading of the bot.
// trackWidth represents the distance between the centers of the two wheels.

TankKinematics odometry = new TankKinematics(x, y, theta, trackWidth);
```

Updating:
```
// left represents the input of the left encoder.
// right represents the input of the right encoder.

odometry.updateMeasurements(left, right);
```
### Point Following

If you only need your robot to find the shortest path between point a and point b, then it's best to use point following.
To achieve that you need a localization system and a follower.

Initialization:
```
// odometry represents a localization system that implements PositionProvider.
// Use this if you do not plan to tune the PID controller.

PathFollower follower = new Follower(odometry);

// Alternitively, use this.
// odometry represents a localization system that implements PositionProvider.
// controller represents an instance of the PIDController class.

PathFollower follower = new PathFollower(odometry, controller);
```

To instantiate a point use this syntax.
```
// x represents the position of the point in the x axis.
// y represents the position of the point in the y axis.

Point point = new Point(x, y);
```

Following:
```
// point represents the point we created earlier.

follower.followPoint(point);

// Storing the motor powers.

double[] powers = follower.getCalculatedPowers();

// lm represents the left motor of our robot.
// rm represents the right motor of our robot.

lm.setPower(powers[0] + powers[1]);
rm.setPower(powers[0] - powers[1]);
```

### Point Sequencing

If you'd like your robot to go to some certain point in a successive manner, use a `PointSequence`.
To follow it, use a follower and initialize in the way previously mentioned.

Initialization:
```
// p1 - p5 represent the points of the sequence.

PointSequence seq = new PointSequence(p1, p2, p3, p4, p5);

// follower represents the PathFollower we created earlier.
// tolerance represents how close the robot needs to get to a point to switch to the next point of the sequence.

follower.setDistanceErrorTolerance(tolerance);
```

Following:
```
// seq represents the sequence we created earlier.

follower.followPointSequence(seq);

// Storing the motor powers.

double[] powers = follower.getCalculatedPowers();

// lm represents the left motor of our robot.
// rm represents the right motor of our robot.

lm.setPower(powers[0] + powers[1]);
rm.setPower(powers[0] - powers[1]);
```

### Paths

If you need your robot precisely follow a specific path without caring about going to certain points, use path following.
Path following uses a follower, same as all the previous methods, and a `Path`.
It uses an algorithm call 'Pure Pursuit' to navigate smoothly around the given points of the path.

Initialization:
```
// p1 - p5 represent the points of the path.

Path path = new Path(p1, p2, p3, p4, p5);

// follower represents the PathFollower we created earlier.
// admissibleError represents how close the robot needs to get to the last point to mark the path as finished.
// lookAheadDistance represents the radius of the circle used in the circle line intersection check.
// The only thing you thing you need to know about a look ahead distance is that a higher value will result in a much smoother following of the path at the cost of some accuracy.

follower.purePurSuitSetUp(lookAheadDistance, admissibleError);
```

Following:
```
// path represents the path we created earlier.

follower.followPath(path);

// Storing the motor powers.

double[] powers = follower.getCalculatedPowers();

// lm represents the left motor of our robot.
// rm represents the right motor of our robot.

lm.setPower(powers[0] + powers[1]);
rm.setPower(powers[0] - powers[1]);
```

## FAQ

### Possible answers
- Coming soon
- Definitely not
- Of course
- It is going to get added very soon but, it's still easy to be done in the current state of the library
- Mecanum drivetrain support soon™
- Yes, the fact that the measurements and the calculations for the localizers is indeed atrocious but, it's one of my first priorities to fix it

## Notes

Part of the code for path following was made with the help of the tutorials of Gluten Free on the subject.
They are very interesting and can teach you a lot about the backend of the library. If you are interested [here](https://www.youtube.com/watch?v=gxnZ6Q8xo-o) is the link.

This library is dedicated to my old teammate Foivos, who helped code the autonomous in my rookie year. :)