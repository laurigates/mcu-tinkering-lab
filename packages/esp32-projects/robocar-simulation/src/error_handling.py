"""
Error Handling and Graceful Degradation Framework

This module provides comprehensive error handling, resilience, and graceful
degradation capabilities for the robot simulation system.
"""

import functools
import logging
import threading
import time
import traceback
from collections.abc import Callable
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any


class ErrorSeverity(Enum):
    """Error severity levels"""

    LOW = "low"  # Minor issues, continue operation
    MEDIUM = "medium"  # Degraded operation, fallback mode
    HIGH = "high"  # Significant issues, limited operation
    CRITICAL = "critical"  # System failure, emergency stop


class ComponentStatus(Enum):
    """Component operational status"""

    OPERATIONAL = "operational"
    DEGRADED = "degraded"
    FAILED = "failed"
    OFFLINE = "offline"
    RECOVERING = "recovering"


@dataclass
class ErrorEvent:
    """Error event record"""

    timestamp: float
    component: str
    error_type: str
    severity: ErrorSeverity
    message: str
    exception: str | None = None
    recovery_attempted: bool = False
    recovery_successful: bool = False


@dataclass
class ComponentHealth:
    """Component health status"""

    name: str
    status: ComponentStatus
    last_error: ErrorEvent | None = None
    error_count: int = 0
    last_success: float | None = None
    uptime_start: float | None = None


class SimulationErrorHandler:
    """
    Central error handling and resilience system for robot simulation
    """

    def __init__(self, log_file: str | None = None):
        # Setup logging
        self.logger = self._setup_logging(log_file)

        # Error tracking
        self.error_history: list[ErrorEvent] = []
        self.component_health: dict[str, ComponentHealth] = {}
        self.max_error_history = 1000

        # Recovery strategies
        self.recovery_strategies: dict[str, list[Callable]] = {}
        self.recovery_attempts: dict[str, int] = {}
        self.max_recovery_attempts = 3

        # Circuit breaker pattern
        self.circuit_breakers: dict[str, dict] = {}

        # System status
        self.system_operational = True
        self.degraded_mode = False
        self.emergency_stop = False

        # Lock for thread safety
        self.lock = threading.RLock()

        self.logger.info("Error handling system initialized")

    def _setup_logging(self, log_file: str | None) -> logging.Logger:
        """Setup logging configuration"""
        logger = logging.getLogger("simulation_errors")
        logger.setLevel(logging.DEBUG)

        # Remove existing handlers
        for handler in logger.handlers[:]:
            logger.removeHandler(handler)

        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        console_handler.setFormatter(console_format)
        logger.addHandler(console_handler)

        # File handler if specified
        if log_file:
            file_handler = logging.FileHandler(log_file)
            file_handler.setLevel(logging.DEBUG)
            file_format = logging.Formatter(
                "%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s"
            )
            file_handler.setFormatter(file_format)
            logger.addHandler(file_handler)

        return logger

    def register_component(self, component_name: str):
        """Register a component for health monitoring"""
        with self.lock:
            if component_name not in self.component_health:
                self.component_health[component_name] = ComponentHealth(
                    name=component_name, status=ComponentStatus.OFFLINE, uptime_start=time.time()
                )
                self.logger.info(f"Registered component: {component_name}")

    def report_component_success(self, component_name: str):
        """Report successful operation of a component"""
        with self.lock:
            if component_name not in self.component_health:
                self.register_component(component_name)

            health = self.component_health[component_name]
            health.last_success = time.time()

            # Update status if recovering
            if health.status in [ComponentStatus.FAILED, ComponentStatus.RECOVERING]:
                health.status = ComponentStatus.OPERATIONAL
                self.logger.info(f"Component {component_name} recovered")
            elif health.status == ComponentStatus.OFFLINE:
                health.status = ComponentStatus.OPERATIONAL
                self.logger.info(f"Component {component_name} came online")

    def handle_error(
        self,
        component: str,
        error_type: str,
        message: str,
        exception: Exception | None = None,
        severity: ErrorSeverity = ErrorSeverity.MEDIUM,
    ) -> bool:
        """
        Handle an error event and attempt recovery

        Returns:
            True if error was handled and system can continue
            False if error requires system shutdown
        """
        with self.lock:
            # Create error event
            error_event = ErrorEvent(
                timestamp=time.time(),
                component=component,
                error_type=error_type,
                severity=severity,
                message=message,
                exception=str(exception) if exception else None,
            )

            # Add to history
            self.error_history.append(error_event)
            if len(self.error_history) > self.max_error_history:
                self.error_history.pop(0)

            # Update component health
            if component not in self.component_health:
                self.register_component(component)

            health = self.component_health[component]
            health.last_error = error_event
            health.error_count += 1

            # Log error
            log_level = self._severity_to_log_level(severity)
            self.logger.log(log_level, f"{component}: {error_type} - {message}")
            if exception:
                self.logger.debug(f"Exception details: {traceback.format_exc()}")

            # Handle based on severity
            return self._handle_error_by_severity(error_event)

    def _severity_to_log_level(self, severity: ErrorSeverity) -> int:
        """Convert error severity to logging level"""
        mapping = {
            ErrorSeverity.LOW: logging.INFO,
            ErrorSeverity.MEDIUM: logging.WARNING,
            ErrorSeverity.HIGH: logging.ERROR,
            ErrorSeverity.CRITICAL: logging.CRITICAL,
        }
        return mapping.get(severity, logging.WARNING)

    def _handle_error_by_severity(self, error: ErrorEvent) -> bool:
        """Handle error based on severity level"""
        component = error.component

        if error.severity == ErrorSeverity.LOW:
            # Log and continue
            return True

        elif error.severity == ErrorSeverity.MEDIUM:
            # Attempt recovery, possibly degrade
            return self._attempt_recovery(error)

        elif error.severity == ErrorSeverity.HIGH:
            # Serious issue, definitely degrade, attempt recovery
            self._set_component_status(component, ComponentStatus.DEGRADED)
            self.degraded_mode = True
            return self._attempt_recovery(error)

        elif error.severity == ErrorSeverity.CRITICAL:
            # System failure, emergency stop
            self._set_component_status(component, ComponentStatus.FAILED)
            self.emergency_stop = True
            self.system_operational = False
            self.logger.critical(f"EMERGENCY STOP: Critical error in {component}")
            return False

        return True

    def _attempt_recovery(self, error: ErrorEvent) -> bool:
        """Attempt to recover from an error"""
        component = error.component

        # Check circuit breaker
        if self._is_circuit_breaker_open(component):
            self.logger.warning(f"Circuit breaker open for {component}, skipping recovery")
            return False

        # Check recovery attempt limit
        if self.recovery_attempts.get(component, 0) >= self.max_recovery_attempts:
            self.logger.error(f"Max recovery attempts reached for {component}")
            self._set_component_status(component, ComponentStatus.FAILED)
            return False

        # Attempt recovery
        if component in self.recovery_strategies:
            self._set_component_status(component, ComponentStatus.RECOVERING)
            self.recovery_attempts[component] = self.recovery_attempts.get(component, 0) + 1

            for strategy in self.recovery_strategies[component]:
                try:
                    self.logger.info(f"Attempting recovery for {component}")
                    success = strategy(error)
                    if success:
                        error.recovery_attempted = True
                        error.recovery_successful = True
                        self._set_component_status(component, ComponentStatus.OPERATIONAL)
                        self.recovery_attempts[component] = 0  # Reset counter
                        self.logger.info(f"Recovery successful for {component}")
                        return True
                except Exception as e:
                    self.logger.error(f"Recovery strategy failed for {component}: {e}")

            # All recovery strategies failed
            error.recovery_attempted = True
            error.recovery_successful = False
            self._set_component_status(component, ComponentStatus.FAILED)
            self.logger.error(f"All recovery strategies failed for {component}")

        return False

    def _set_component_status(self, component: str, status: ComponentStatus):
        """Update component status"""
        if component not in self.component_health:
            self.register_component(component)

        old_status = self.component_health[component].status
        self.component_health[component].status = status

        if old_status != status:
            self.logger.info(f"Component {component} status: {old_status.value} -> {status.value}")

    def register_recovery_strategy(self, component: str, strategy: Callable[[ErrorEvent], bool]):
        """Register a recovery strategy for a component"""
        if component not in self.recovery_strategies:
            self.recovery_strategies[component] = []

        self.recovery_strategies[component].append(strategy)
        self.logger.info(f"Registered recovery strategy for {component}")

    def _is_circuit_breaker_open(self, component: str) -> bool:
        """Check if circuit breaker is open for component"""
        if component not in self.circuit_breakers:
            return False

        breaker = self.circuit_breakers[component]
        current_time = time.time()

        # Check if breaker should reset
        if current_time - breaker["last_failure"] > breaker["timeout"]:
            breaker["failure_count"] = 0
            return False

        return breaker["failure_count"] >= breaker["threshold"]

    def trip_circuit_breaker(self, component: str, threshold: int = 5, timeout: float = 60.0):
        """Trip circuit breaker for component"""
        current_time = time.time()

        if component not in self.circuit_breakers:
            self.circuit_breakers[component] = {
                "failure_count": 0,
                "threshold": threshold,
                "timeout": timeout,
                "last_failure": current_time,
            }

        breaker = self.circuit_breakers[component]
        breaker["failure_count"] += 1
        breaker["last_failure"] = current_time

        if breaker["failure_count"] >= threshold:
            self.logger.warning(f"Circuit breaker tripped for {component}")

    def get_system_status(self) -> dict[str, Any]:
        """Get overall system status"""
        with self.lock:
            return {
                "operational": self.system_operational,
                "degraded_mode": self.degraded_mode,
                "emergency_stop": self.emergency_stop,
                "components": {
                    name: asdict(health) for name, health in self.component_health.items()
                },
                "total_errors": len(self.error_history),
                "recent_errors": len(
                    [e for e in self.error_history if time.time() - e.timestamp < 300]
                ),  # Last 5 minutes
            }

    def get_component_health(self, component: str) -> ComponentHealth | None:
        """Get health status of specific component"""
        return self.component_health.get(component)

    def reset_component(self, component: str):
        """Reset component error state"""
        with self.lock:
            if component in self.component_health:
                self.component_health[component].error_count = 0
                self.component_health[component].last_error = None
                self._set_component_status(component, ComponentStatus.OPERATIONAL)
                self.logger.info(f"Reset component {component}")

            if component in self.recovery_attempts:
                self.recovery_attempts[component] = 0

    def emergency_shutdown(self, reason: str):
        """Trigger emergency shutdown"""
        with self.lock:
            self.emergency_stop = True
            self.system_operational = False
            self.logger.critical(f"EMERGENCY SHUTDOWN: {reason}")

            # Mark all components as failed
            for component in self.component_health:
                self._set_component_status(component, ComponentStatus.FAILED)


def resilient_operation(
    component: str,
    error_handler: SimulationErrorHandler,
    error_type: str = "operation_failed",
    severity: ErrorSeverity = ErrorSeverity.MEDIUM,
):
    """
    Decorator for resilient operation with automatic error handling
    """

    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            try:
                result = func(*args, **kwargs)
                error_handler.report_component_success(component)
                return result
            except Exception as e:
                handled = error_handler.handle_error(
                    component=component,
                    error_type=error_type,
                    message=f"Function {func.__name__} failed: {str(e)}",
                    exception=e,
                    severity=severity,
                )

                if not handled:
                    raise  # Re-raise if not handled

                return None  # Return None for handled errors

        return wrapper

    return decorator


def async_resilient_operation(
    component: str,
    error_handler: SimulationErrorHandler,
    error_type: str = "async_operation_failed",
    severity: ErrorSeverity = ErrorSeverity.MEDIUM,
):
    """
    Decorator for resilient async operation with automatic error handling
    """

    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        async def wrapper(*args, **kwargs):
            try:
                result = await func(*args, **kwargs)
                error_handler.report_component_success(component)
                return result
            except Exception as e:
                handled = error_handler.handle_error(
                    component=component,
                    error_type=error_type,
                    message=f"Async function {func.__name__} failed: {str(e)}",
                    exception=e,
                    severity=severity,
                )

                if not handled:
                    raise  # Re-raise if not handled

                return None  # Return None for handled errors

        return wrapper

    return decorator


# Global error handler instance
_global_error_handler: SimulationErrorHandler | None = None
_global_error_handler_lock = threading.Lock()


def get_error_handler() -> SimulationErrorHandler:
    """Get or create global error handler (thread-safe)"""
    global _global_error_handler
    if _global_error_handler is None:
        with _global_error_handler_lock:
            if _global_error_handler is None:
                _global_error_handler = SimulationErrorHandler("simulation_errors.log")
    return _global_error_handler


def initialize_error_handling(log_file: str | None = None) -> SimulationErrorHandler:
    """Initialize global error handling system"""
    global _global_error_handler
    _global_error_handler = SimulationErrorHandler(log_file)
    return _global_error_handler


if __name__ == "__main__":
    # Example usage
    error_handler = SimulationErrorHandler()

    # Register some components
    error_handler.register_component("motor_controller")
    error_handler.register_component("camera")
    error_handler.register_component("ai_processor")

    # Example recovery strategy
    def motor_recovery(error: ErrorEvent) -> bool:
        print(f"Attempting motor recovery for error: {error.message}")
        # Simulate recovery logic
        return True  # Recovery successful

    error_handler.register_recovery_strategy("motor_controller", motor_recovery)

    # Test error handling
    error_handler.handle_error(
        "motor_controller", "pwm_failure", "PWM signal lost", severity=ErrorSeverity.MEDIUM
    )
    error_handler.handle_error(
        "camera", "frame_capture_failed", "No frames received", severity=ErrorSeverity.HIGH
    )

    # Show system status
    status = error_handler.get_system_status()
    print(f"System Status: {status}")
