# Drive settings inspection (no motion)

Connect USB and motor power with cube removed; close other motor programs.
Run `leap_drive_readonly.cmd`. The program reads all16 motors and requires torque
OFF at the beginning and end. It never requests torque ON/OFF, a target, a gain
change, or an EEPROM write. No SPACE start is required.

Extra registers are read twice and inconsistent values stop the inspection.
Results include PWM Limit36, Current Limit38, position limits48/52, PID80/82/84,
Goal PWM100, Goal Current102, Profile Acceleration108 and Velocity112. Output:
`leap_drive_readonly_result.json`. Return the terminal text after it finishes.

The current checked movement uses operating mode5. Its position limits48/52 do
not constrain goals. PWM limits can constrain output independently of current.
These are restored/off settings; a motor's active PWM saturation during the earlier
movement cannot be reconstructed from this read. No automatic changes follow.
Reference: https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/
