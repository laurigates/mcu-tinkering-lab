# Add comprehensive integration tests

**Labels**: `enhancement`, `testing`, `priority: high`

## Problem
Currently have unit tests but no integration tests verifying end-to-end functionality. Need to validate:
- WebSocket communication works correctly
- Serial communication mocking
- Full simulation scenarios
- Performance benchmarks

## Proposed Test Coverage

### WebSocket Integration Tests
- [ ] Start WebSocket server in test
- [ ] Connect test client to ws://localhost:8765
- [ ] Send motor command JSON, verify robot moves
- [ ] Send servo command, verify camera angle changes
- [ ] Request robot state, verify JSON response format
- [ ] Test connection handling (connect, disconnect, reconnect)
- [ ] Test error handling (invalid JSON, bad commands)

### Serial Communication Tests
- [ ] Mock serial port for testing
- [ ] Test I2C message encoding/decoding
- [ ] Verify CRC8 checksum calculation
- [ ] Test all message types (MOVE, SERVO, SOUND, etc.)
- [ ] Test serial data parsing

### End-to-End Scenarios
- [ ] Test: Start robot, send commands, verify trajectory
- [ ] Test: Square path following (4 sides, 90° turns)
- [ ] Test: Collision detection with obstacles
- [ ] Test: Sensor readings update correctly
- [ ] Test: Camera simulation captures frames
- [ ] Test: WiFi/OTA simulation state machines

### Performance Benchmarks
- [ ] Measure simulation update rate (target: 100Hz)
- [ ] Measure WebSocket latency (target: <10ms)
- [ ] Measure memory usage over time (check for leaks)
- [ ] Stress test: 1000 commands in 10 seconds

## Test Framework
- Use pytest with pytest-asyncio for async tests
- Use websockets library for client testing
- Use unittest.mock for serial port mocking
- Add performance decorators/fixtures

## Files to Create
- `tests/integration/test_websocket_bridge.py`
- `tests/integration/test_serial_communication.py`
- `tests/integration/test_scenarios.py`
- `tests/performance/test_benchmarks.py`
- `tests/conftest.py` - Shared fixtures

## Impact
- **Priority**: High ⭐⭐⭐
- **Effort**: Medium-High (2-3 hours)
- **Benefit**: Confidence in real hardware integration, catch bugs early

## Success Criteria
- [ ] All integration tests passing
- [ ] Can run `pytest tests/integration/` successfully
- [ ] CI/CD can run tests in headless mode
- [ ] Code coverage >80% for communication code

## Related
- Validates fixes from PR #43
- Prepares for real ESP32 hardware testing
