#ifndef MYCAMERACONTROLLER_H
#define MYCAMERACONTROLLER_H

#include <Qt3DExtras/QAbstractCameraController>
#include <QPoint>
#include <QSize>
#include <Qt3DCore/QTransform>

class CustomCameraController : public Qt3DExtras::QAbstractCameraController
{
    Q_OBJECT
public:
    Q_PROPERTY(QSize windowSize READ windowSize WRITE setWindowSize NOTIFY windowSizeChanged)
    Q_PROPERTY(float rotationSpeed READ rotationSpeed WRITE setRotationSpeed NOTIFY rotationSpeedChanged)
    Q_PROPERTY(float translationSpeed READ translationSpeed WRITE setTranslationSpeed NOTIFY translationSpeedChanged)

    CustomCameraController(Qt3DCore::QNode *parent = nullptr);

    QSize windowSize() const
    {
        return m_windowSize;
    }

    float rotationSpeed() const
    {
        return m_rotationSpeed;
    }

    float translationSpeed() const
    {
        return m_translationSpeed;
    }

public slots:
    void setWindowSize(QSize windowSize)
    {
        if (m_windowSize == windowSize)
            return;

        m_windowSize = windowSize;
        emit windowSizeChanged(m_windowSize);
    }

    void setRotationSpeed(float rotationSpeed)
    {
        if (qFuzzyCompare(m_rotationSpeed, rotationSpeed))
            return;

        m_rotationSpeed = rotationSpeed;
        emit rotationSpeedChanged(m_rotationSpeed);
    }

    void setTranslationSpeed(float translationSpeed)
    {
        if (qFuzzyCompare(m_translationSpeed, translationSpeed))
            return;

        m_translationSpeed = translationSpeed;
        emit translationSpeedChanged(m_translationSpeed);
    }

signals:
    void windowSizeChanged(QSize windowSize);
    void rotationSpeedChanged(float rotationSpeed);
    void translationSpeedChanged(float translationSpeed);

protected:
    void moveCamera(const Qt3DExtras::QAbstractCameraController::InputState &state, float dt) override;

private:
    QPoint m_mouseLastPosition, m_mouseCurrentPosition;
    QSize m_windowSize;
    float m_rotationSpeed = .2f;
    float m_translationSpeed = .01f;
};

#endif // MYCAMERACONTROLLER_H
