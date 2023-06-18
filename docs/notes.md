
## Troubleshooting

### Motor settings
Setting motor info e.g. IDs needs torque to be disabled.
For example, to set the ID of motor 1 to 7 in using `r0b0.scripts.motor_calib.py`:
```
set_param('torque_enable',{1:False})
set_param('id',{1:7})
```