from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    loadPrcFileData,
    WindowProperties,
    Point3,
    Filename,
    DirectionalLight,
    AmbientLight,
    CardMaker,
    LineSegs
)
from direct.gui.OnscreenText import OnscreenText
import numpy as np

# (Optional) run offscreen if you need to batch‑convert without a window
# loadPrcFileData('', 'window-type offscreen')

class UAVVisualizer(ShowBase):
    def __init__(self):
        super().__init__()

        # Sky‑blue background
        self.setBackgroundColor(0.53, 0.81, 0.92, 1)

        # 1) Load the converted EGG model
        #    Make sure you've done:
        #      obj2egg visualize/egg.obj visualize/egg.egg
        self.vehicle = self.loader.loadModel("visualize/egg.obj")
        self.vehicle.reparentTo(self.render)
        self.vehicle.setScale(100)      # Tweak this to taste
        self.vehicle.setPos(0, 0, 1000)

        # 2) Draw local axes on the vehicle so you can see its orientation
        self._attach_axes()

        # 3) On‑screen text for Roll/Pitch/Yaw
        self.angle_text = OnscreenText(
            text="", pos=(-1.3, 0.9), scale=0.06, mayChange=True
        )

        # State bookkeeping
        self.latest_state = None
        self.disableMouse()

        # Camera modes
        self.camera_modes = ['third_person', 'fpv', 'follow']
        self.current_camera_mode = 0
        self.set_camera_view()

        # Controls
        self.accept('c', self.switch_camera_view)
        self.accept('wheel_up', self.zoom_in)
        self.accept('wheel_down', self.zoom_out)

        # Tasks
        self.taskMgr.add(self.update_task, "UpdateTask")
        self.taskMgr.add(self.update_camera_task, "CameraFollowTask")

        # Environment & lighting
        self._create_ground()
        self._setup_lighting()

    def _attach_axes(self):
        """Attach RGB axes of length 200 units to the vehicle."""
        ls = LineSegs()
        ls.setThickness(2.0)
        # X = red
        ls.setColor(1, 0, 0, 1)
        ls.moveTo(0, 0, 0); ls.drawTo(200, 0, 0)
        # Y = green
        ls.setColor(0, 1, 0, 1)
        ls.moveTo(0, 0, 0); ls.drawTo(0, 200, 0)
        # Z = blue
        ls.setColor(0, 0, 1, 1)
        ls.moveTo(0, 0, 0); ls.drawTo(0, 0, 200)

        axes_np = self.vehicle.attachNewNode(ls.create())
        axes_np.setTransparency(True)

    def _create_ground(self):
        """Create a simple green ground plane for reference."""
        cm = CardMaker("ground")
        cm.setFrame(-10000, 10000, -10000, 10000)
        ground = self.render.attachNewNode(cm.generate())
        ground.setPos(0, 0, 0)
        ground.lookAt(0, 0, -1)
        ground.setColor(0.4, 0.8, 0.4, 1)

    def _setup_lighting(self):
        """Add one directional light and a little ambient fill."""
        dlight = DirectionalLight('dlight')
        dlight.setColor((0.8, 0.8, 0.8, 1))
        dlnp = self.render.attachNewNode(dlight)
        dlnp.setHpr(0, -60, 0)
        self.render.setLight(dlnp)

        alight = AmbientLight('alight')
        alight.setColor((0.2, 0.2, 0.2, 1))
        alnp = self.render.attachNewNode(alight)
        self.render.setLight(alnp)

    def update_task(self, task):
        """Apply the latest_state to the vehicle and update the angle text."""
        if self.latest_state is not None:
            try:
                if not np.all(np.isfinite(self.latest_state)):
                    print("Warning: Non-finite values in state, skipping frame.")
                    self.latest_state = None
                    return task.cont

                x, y, z = self.latest_state[0:3]
                phi, theta, psi = self.latest_state[6:9]

                # Position & orientation
                self.vehicle.setPos(x, y, z)
                self.vehicle.setHpr(
                    np.degrees(psi),
                    np.degrees(theta),
                    np.degrees(phi)
                )

                # Update on-screen text
                self.angle_text.setText(
                    f"R:{np.degrees(phi):.1f}°  "
                    f"P:{np.degrees(theta):.1f}°  "
                    f"Y:{np.degrees(psi):.1f}°"
                )

                self.latest_state = None

            except Exception as e:
                print(f"Error updating visualization: {e}")
                self.latest_state = None

        return task.cont

    def update_camera_task(self, task):
        """If in 'follow' mode, manually position the camera each frame."""
        if self.camera_modes[self.current_camera_mode] == "follow":
            offset = Point3(0, -300, 100)
            cam_pos = self.vehicle.getPos(self.render) + offset
            self.camera.setPos(cam_pos)
            self.camera.lookAt(self.vehicle)
        return task.cont

    def update_visualization(self, state):
        """Call this externally with your new state array."""
        self.latest_state = state
        # Force one render pass
        self.taskMgr.step()

    def set_camera_view(self):
        mode = self.camera_modes[self.current_camera_mode]
        if mode == 'third_person':
            self.camera.reparentTo(self.vehicle)
            self.camera.setPos(0, -200, 60)
            self.camera.lookAt(self.vehicle)
        elif mode == 'fpv':
            self.camera.reparentTo(self.vehicle)
            self.camera.setPos(0, 10, 5)
            self.camera.lookAt(self.vehicle.getPos() + self.vehicle.getQuat().getForward())
        elif mode == 'follow':
            # Must be parented to render for manual updates
            self.camera.reparentTo(self.render)

    def switch_camera_view(self):
        self.current_camera_mode = (self.current_camera_mode + 1) % len(self.camera_modes)
        self.set_camera_view()

    def zoom_in(self):
        current_fov = self.camLens.getFov()[0]
        self.camLens.setFov(max(current_fov - 5, 30))

    def zoom_out(self):
        current_fov = self.camLens.getFov()[0]
        self.camLens.setFov(min(current_fov + 5, 90))


if __name__ == "__main__":
    viz = UAVVisualizer()
    viz.run()
