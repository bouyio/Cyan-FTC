[![](https://jitpack.io/v/bouyio/Cyan-FTC.svg)](https://jitpack.io/#bouyio/Cyan-FTC)

# Cyan-FTC

*A lightweight motion-planning library for **FIRST Tech Challenge** teams.*

<div align="center">

[Getting Started](#-getting-started) •
[Features](#-features) •
[Installation](#-installation) •
[Usage Examples](#-usage-examples) •
[FAQ](#-faq) •
[Contributing](#-contributing) •
[License](#-license)

</div>

---

## ✨ Overview
Cyan-FTC is a community-driven project **independent of any single FTC team**.  
Its mission is to make advanced autonomous movement *approachable* for rookie teams while still being *powerful* enough for veterans.

> **Using Cyan-FTC at the Greek FTC Championship?**  
> Come say hi—I’d love to hear about your experience and help with any issues.

---

## 💡 Features
| Category | Highlights |
| -------- | ---------- |
| **Localization** | Plug-and-play odometry (`GyroTankOdometry`, `TankKinematics`) or bring your own by implementing `PositionProvider`. |
| **Path Following** | Pure Pursuit with configurable look-ahead, distance tolerance, and PID gains. |
| **Unit-Aware Math** | Smart `Distance`, `Vector`, and `Point` classes—convert seamlessly between *in*, *cm*, *m*, *ticks*, etc. |
| **Modularity First** | Works with tank, 2-wheel, 3-wheel, and (soon™) mecanum drivetrains. |
| **Rookie Friendly** | Clean API, extensive Javadoc, and annotated examples. |

---

## 📦 Installation

### Prerequisites
- Java Development Kit (JDK 8+ recommended)  
- Android-compatible IDE (Android Studio, IntelliJ, VS Code, etc.)  
- Official **FTC SDK**: [github.com/FIRST-Tech-Challenge/FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController)

### Step-by-Step

1. **Add JitPack to your root `build.dependencies.gradle`:**
   ```gradle
   repositories {
       // …other repositories…
       maven { url "https://jitpack.io" }
   }
   ```

2. **Add Cyan-FTC to `TeamCode/build.gradle`:**
   ```gradle
   dependencies {
       implementation "com.github.bouyio:Cyan-FTC:1.1"
   }
   ```

3. **Sync Gradle & build.** Done! 🎉

---

## 🏁 Getting Started

The quickest way to explore Cyan-FTC is via the **examples folder** (coming soon).  
Below is a high-level tour—see [📚 Javadocs](#) (link once published) for full details.

### Localization

1. **Create a measurement provider**
   ```java
   DoubleSupplier leftEnc  = leftEncoder::getCurrentPosition;
   DoubleSupplier rightEnc = rightEncoder::getCurrentPosition;
   DoubleSupplier heading  = imu::getAngle;            // degrees (CCW +)

   double ticksPerMeter = /* 2 π r / ticksPerRev */;
   var provider = new GyroTankMeasurementProvider(
           leftEnc, rightEnc, heading, ticksPerMeter);
   ```

2. **Initialize odometry**
   ```java
   SmartVector2 startPos = new SmartVector2(0, 0, DistanceUnit.METER);
   double startHeading  = 0;       // rad

   GyroTankOdometry odometry =
           new GyroTankOdometry(startPos, startHeading, provider);
   ```

3. **Update (automatic or manual)**
   ```java
   odometry.update();   // call in your OpMode loop if manual
   ```

<details>
<summary><strong>No IMU?</strong> Use <code>TankKinematics</code></summary>

```java
DoubleSupplier leftEnc  = leftEncoder::getCurrentPosition;
DoubleSupplier rightEnc = rightEncoder::getCurrentPosition;
double trackWidth       = 0.32;    // meters

var provider = new TankKinematicsMeasurementProvider(
        leftEnc, rightEnc, ticksPerMeter);

TankKinematics odometry =
        new TankKinematics(startPos, startHeading, trackWidth, provider);
```
</details>

### Point Following
```java
PathFollower follower = new PathFollower(odometry);    // default PID
Point goal = new Point(1.2, 0.8, DistanceUnit.METER);

follower.followPoint(goal);

double[] motor = follower.getCalculatedPowers();
leftMotor.setPower(motor[0] + motor[1]);
rightMotor.setPower(motor[0] - motor[1]);
```

### Sequencing & Paths
- `PointSequence` for visiting waypoints in order.
- `Path` + `purePursuitSetup()` for smooth curves (Pure Pursuit).

---

## 🔄 New in v1.1
- **Mid-cycle localization updates**—inject position corrections on the fly.
- Unit-aware `Distance`, `Vector`, `Point` classes.
- `PathFollower` now resizes points automatically based on distance units.

---

## 🗂 Usage Examples
> Full example OpModes coming soon. Meanwhile, check the **snippets above** and the `examples` package in source.

---

## ❓ FAQ
| Question | Answer |
| -------- | ------ |
| “When will mecanum be supported?” | Soon™ |
| “Can I use XYZ drivetrain?” | Of course—implement `PositionProvider`. |
| “Is there documentation?” | Javadoc is generated each release. Detailed guides are WIP. |
| More Qs? | Open an [issue](https://github.com/bouyio/Cyan-FTC/issues) or ping me on Discord. |

---

## 🤝 Contributing
Contributions are **welcome**! Please:

1. Fork & create a feature branch.
2. Follow the code-style guidelines (KTlint/Checkstyle config coming soon).
3. Make sure `./gradlew check` passes.
4. Open a PR—fill in the template.

---

## 📝 License
This project is MIT-licensed.  
See [LICENSE](LICENSE) for details.

---

## 📜 Acknowledgements
Path-following logic was inspired by Gluten Free’s excellent tutorials—watch them [here](https://www.youtube.com/watch?v=gxnZ6Q8xo-o).  

> Dedicated to Foivos, my rookie-year teammate who made autonomous ***fun***.
