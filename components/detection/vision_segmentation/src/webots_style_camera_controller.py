from PySide6.QtGui import QVector3D
from PySide6.Qt3DRender import Qt3DRender
from PySide6.Qt3DExtras import Qt3DExtras


class WebotsStyleCameraController(Qt3DExtras.QAbstractCameraController):
    def __init__(self, parent=None):
        super().__init__(parent)

    def moveCamera(self, state, dt):
        cam = self.camera()
        if not cam:
            return

        if state.leftMouseButtonActive and not state.rightMouseButtonActive:
            cam.panAboutViewCenter(state.rxAxisValue * self.lookSpeed() * dt)
            cam.tiltAboutViewCenter(-state.ryAxisValue * self.lookSpeed() * dt)

        if state.rightMouseButtonActive:
            cam.translate(QVector3D(-state.rxAxisValue * self.linearSpeed() * dt,
                                    -state.ryAxisValue * self.linearSpeed() * dt,
                                    0.0),
                          Qt3DRender.QCamera.TranslateViewCenter)

        if abs(state.tzAxisValue) > 0.0:
            cam.translate(QVector3D(0.0, 0.0, state.tzAxisValue * self.linearSpeed() * dt),
                          Qt3DRender.QCamera.DontTranslateViewCenter)
