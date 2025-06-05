#include "customcameracontroller.h"
#include <Qt3DRender/QCamera>
#include <Qt3DInput/QAction>
#include <Qt3DInput/QActionInput>
#include <Qt3DInput/QAxis>
#include <Qt3DInput/QAnalogAxisInput>
#include <Qt3DInput/QMouseDevice>
#include <Qt3DInput/QKeyboardDevice>
#include <Qt3DInput/QMouseHandler>
#include <Qt3DInput/QKeyboardHandler>
#include <Qt3DCore/QTransform>
#include <QtMath>

CustomCameraController::CustomCameraController(Qt3DCore::QNode *parent)
    : Qt3DExtras::QAbstractCameraController(parent)
{
    Qt3DInput::QMouseHandler *mouseHandler = new Qt3DInput::QMouseHandler(this);
    mouseHandler->setSourceDevice(mouseDevice());

    QObject::connect(mouseHandler, &Qt3DInput::QMouseHandler::pressed,
                     [this](Qt3DInput::QMouseEvent *pressedEvent) {
                         pressedEvent->setAccepted(true);
                         m_mouseLastPosition = QPoint(pressedEvent->x(), pressedEvent->y());
                         m_mouseCurrentPosition = m_mouseLastPosition;
                     });

    QObject::connect(mouseHandler, &Qt3DInput::QMouseHandler::positionChanged,
                     [this](Qt3DInput::QMouseEvent *positionChangedEvent) {
                         positionChangedEvent->setAccepted(true);
                         m_mouseCurrentPosition = QPoint(positionChangedEvent->x(),
                                                         positionChangedEvent->y());
                     });
}

void CustomCameraController::moveCamera(const Qt3DExtras::QAbstractCameraController::InputState &state, float dt)
{
    auto theCamera = camera();

    if (theCamera == nullptr)
        return;

    auto ls = linearSpeed();

    if (state.leftMouseButtonActive) {
        auto offset = m_mouseCurrentPosition - m_mouseLastPosition;
        float angleX = offset.y() * m_rotationSpeed;
        float angleY = offset.x() * m_rotationSpeed;

        theCamera->panAboutViewCenter(angleY);
        theCamera->tiltAboutViewCenter(angleX);
    } else if (state.rightMouseButtonActive) {
        auto offset = m_mouseCurrentPosition - m_mouseLastPosition;
        if (offset.manhattanLength() > 0) {
            float translateX = -offset.x() / float(m_windowSize.width()) * m_translationSpeed;
            float translateY = offset.y() / float(m_windowSize.height()) * m_translationSpeed;
            theCamera->translate(QVector3D(translateX, translateY, 0));
        }

    } else if (dt != 0) {
        theCamera->translate(QVector3D(state.txAxisValue * ls,
                                       state.tyAxisValue * ls,
                                       state.tzAxisValue * ls) * dt,
                             Qt3DRender::QCamera::DontTranslateViewCenter);
    }

    m_mouseLastPosition = m_mouseCurrentPosition;
}
