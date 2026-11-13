"""Robocar simulation.

``robocar_sim.plant`` is the deterministic plant the behaviour twin steps in
lockstep (ADR-036); it needs numpy, pymunk and pyyaml. ``robocar_sim.arena``
owns the arena schema. ``robot_model``, ``motor_controller``, ``error_handling``,
``camera_simulation``, ``genesis_visualizer`` and ``main`` are the legacy demo of
the removed dual-ESP32 car; the camera, the visualiser and ``main`` need the
``viz`` extra.

Nothing is imported here, so importing one module does not pull in another's
dependencies.
"""
