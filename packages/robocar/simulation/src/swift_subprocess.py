import asyncio
import sys
import time
import warnings

try:
    import roboticstoolbox as rtb  # noqa: F401
    import swift  # noqa: F401
    import trimesh
    from spatialmath import SE3
    from swift import Swift
except ImportError as e:
    print(f"Import error: {e}")
    sys.exit(1)

# Suppress warnings
warnings.filterwarnings("ignore", message=".*websockets.*")
warnings.filterwarnings("ignore", message=".*no running event loop.*")


async def run_swift():
    print("Creating Swift instance...")
    env = Swift()

    if "browser" == "browser":
        print("Launching browser mode...")
        env.launch(realtime=True, headless=False, browser="default")
        print("✓ Browser mode active at http://localhost:52000")
    elif "browser" == "visual":
        print("Launching visual mode...")
        env.launch(realtime=True, headless=False)
        print("✓ Visual mode active - GUI window should appear")

    # Add simple test objects
    test_box = trimesh.primitives.Box(extents=[0.15, 0.10, 0.05])
    test_box.visual.face_colors = [100, 150, 200, 255]
    env.add(test_box)

    # Add direction indicator
    arrow = trimesh.primitives.Cylinder(radius=0.01, height=0.08, transform=SE3(0.06, 0, 0.03).A)
    arrow.visual.face_colors = [255, 0, 0, 255]
    env.add(arrow)

    print("✓ Robot visualization created")

    # Keep running
    try:
        while True:
            # Simple animation - rotate the box
            angle = time.time() * 0.5
            transform = SE3.Rz(angle) * SE3(0, 0, 0.025)
            test_box.apply_transform(test_box.transform @ transform.A @ SE3.Rz(-angle / 2).A)
            env.step(0.1)
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        print("Swift subprocess terminated")


if __name__ == "__main__":
    asyncio.run(run_swift())
