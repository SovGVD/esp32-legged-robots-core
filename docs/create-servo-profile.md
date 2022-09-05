## How to
### Calibrate servos (create `servoMainProfile`)
 - Print servo_calib tool and servo horn and install servo into it: circle plate with dots, from 0 to 180deg: 0, 30, 50, 70, 90, 110, 130, 150, 180.
 - Connect servo to alpha channel of first leg.
 - Use CLI and disable HAL (`hal off`).
 - `pwm LEFT alpha 1500` or `pwm LF alpha 1500` (`pwm` - command, `LEFT` or `LF` - leg name, `alpha` - angle name, `1500` microseconds) to set servo to middle. Holes of horn and plate should matched, try to reinstall servo if not.
 - Decrease value to find `minAngle` and `degMin` values for it (start with `800` and decrease it until servo stop move, than return back one step, e.g. set 790 - servo moved, 780 - servo moved, 760 - servo don't move or on 0/180 dot, use previous value `780` and meause angle, e.g. `20`)
 - Do the same to find `maxAngle` and `degMax` but make value and start from 2100 and increase value.
 - Great, now we know our servo limits (or at least what are the limits for lib+servo), time to find more accurate servo positions.
 - Input other microseconds values find proper positions for deg30, deg50...deg130, deg150

Final result could be like that (ESP32PWM + TowerPro MG90D):
```
servoProfile servoMainProfile = {
    20,   // minAngle
    160,  // maxAngle

    780,  // degMin
    890,  // deg30
    1090, // deg50
    1300, // deg70
    1500, // deg90 - middle
    1700, // deg110
    1900, // deg130
    2110, // deg150
    2200  // degMax
};
```

Or slighly different for external PWM controller, e.g. PCA9685 + TowerPro MG90D:
```
servoProfile servoMainProfile = {
    0,    // minAngle
    180,  // maxAngle

    590,  // degMin
    870,  // deg30
    1080, // deg50
    1290, // deg70
    1500, // deg90 - middle
    1710, // deg110
    1890, // deg130
    2100, // deg150
    2200  // degMax
};
```
