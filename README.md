# IMU Example for BeagleBone Black

This uses the LSM9DS0 iNEMO inertial module which contains a 3D accelerometer, 3D gyroscope and 3D magnetometer.

## Installation instructions

```sh
$ git clone https://github.com/timmathews/imu
$ cd imu
$ make
$ ./imu
```

## Output

Currently, the output looks something like this:

```
              Raw Values
    |     X     |     Y     |     Z
----+-----------+-----------+-----------
Acc |  -488.000 |  -537.000 |  1684.000
Gyr |   113.000 |   264.000 |   -88.000
Mag |   344.000 |  -147.000 |  1068.000

              Scaled Values
    |     X     |     Y     |     Z
----+-----------+-----------+-----------
Acc |     0.087 |     0.022 |     0.995
Gyr |   113.000 |   264.000 |   -88.000
Mag |   344.000 |  -147.000 |  1068.000
```

Eventually the output will have the option for JSON formatting and look like this (formatted for humans):

```js
{
  "acc": {
    "x": 0.087,
    "y": 0.022,
    "z": 0.995
  },
  "gyr": {
    "x": 113,
    "y": 264,
    "z": -88
  },
  "mag": {
    "x": 344,
    "y": -147,
    "z": 1068
  },
  "heel": -1.24,
  "hdg": 224.5
}
```

or this:

```
0.087 0.022 0.995 113 264 -88 344 -147 1068 -1.24 224.5
```

Heading is uncorrected magnetic, local variation must be applied. Heel is negative for port and positive for starboard.
Pitch is of limited value and ignored.
