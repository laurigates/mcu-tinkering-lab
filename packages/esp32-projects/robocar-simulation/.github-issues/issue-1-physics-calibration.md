# Fix physics calibration to pass all tests (24/24)

**Labels**: `enhancement`, `testing`, `priority: high`

## Problem
Currently 4 physics tests are failing due to calibration issues. The simulation works, but physics parameters need tuning to match expected behavior.

## Failing Tests
- `test_kinematics_update` - Robot has lateral drift (0.15m instead of ~0)
- `test_differential_drive_kinematics` - Velocity mismatch (0.098 vs expected 0.35 m/s)
- `test_energy_conservation` - Energy drops to near-zero instead of being conserved
- Physics accuracy tests in general

## Current Test Status
- ✅ 18/24 tests passing (75%)
- ⚠️ 2 tests skipped (Genesis/PyTorch - expected)
- ❌ 4 tests failing (physics calibration)

## Tasks
- [ ] Analyze motor friction and resistance parameters
- [ ] Tune `friction_static`, `friction_kinetic`, `friction_viscous` in DCMotor class
- [ ] Calibrate differential drive kinematics equations
- [ ] Fix energy conservation calculation in motor dynamics
- [ ] Validate velocity calculations match expected values
- [ ] Ensure straight-line motion has minimal drift
- [ ] Run full test suite and verify 24/24 passing

## Impact
- **Priority**: High ⭐⭐⭐
- **Effort**: Medium (1-2 hours)
- **Benefit**: Validated physics model, better accuracy, confidence for hardware integration

## Files to Modify
- `src/robot_model.py` - DCMotor class parameters (lines 76-79)
- `tests/test_robot_model.py` - May need to adjust test tolerances if reasonable

## Related
- Original analysis: PR #43
- Test output shows specific parameter values that need adjustment
