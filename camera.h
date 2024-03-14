//
// Created by Xiaopeng Zhang on 3/10/24.
//

#ifndef IPC2DSIM_CAMERA_H
#define IPC2DSIM_CAMERA_H

#include <glad/glad.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "eigen-3.4.0/Eigen/Dense"

enum Camera_Movement
{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 25.0f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;

class Camera
{
public:

    Eigen::Vector3f Position;
    Eigen::Vector3f Front;
    Eigen::Vector3f Up;
    Eigen::Vector3f Right;
    Eigen::Vector3f WorldUp;

    float Yaw;
    float Pitch;

    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    explicit Camera(Eigen::Vector3f position = Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f up = Eigen::Vector3f(0.0f, 1.0f, 0.0f), float yaw = YAW, float pitch = PITCH)\
        : Front(Eigen::Vector3f(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM)
    {
        Position = position;
        WorldUp = up;
        Yaw = yaw;
        Pitch = pitch;
        UpdateCameraVectors();
    }

    Camera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch) : Front(Eigen::Vector3f(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM)
    {
        Position = Eigen::Vector3f(posX, posY, posZ);
        WorldUp = Eigen::Vector3f(upX, upY, upZ);
        Yaw = yaw;
        Pitch = pitch;
        UpdateCameraVectors();
    }

    // returns the view matrix calculated using Euler Angles and the LookAt Matrix
    Eigen::Matrix4f GetViewMatrix()
    {
        Eigen::Vector3f f = Front.normalized();
        Eigen::Vector3f u = Up.normalized();
        Eigen::Vector3f s = f.cross(u).normalized();
        u = s.cross(f);

        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        mat(0, 0) = s.x();
        mat(0, 1) = s.y();
        mat(0, 2) = s.z();
        mat(1, 0) = u.x();
        mat(1, 1) = u.y();
        mat(1, 2) = u.z();
        mat(2, 0) = -f.x();
        mat(2, 1) = -f.y();
        mat(2, 2) = -f.z();
        mat(0, 3) = -s.dot(Position);
        mat(1, 3) = -u.dot(Position);
        mat(2, 3) = f.dot(Position);

        return mat;
    }

    // processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)

    void ProcessKeyboard(Camera_Movement direction, float deltaTime)
    {
        float velocity = MovementSpeed * deltaTime;
        if (direction == FORWARD)
            Position += Front * velocity;
        if (direction == BACKWARD)
            Position -= Front * velocity;
        if (direction == LEFT)
            Position -= Right * velocity;
        if (direction == RIGHT)
            Position += Right * velocity;
    }

    // processes input received from a mouse input system. Expects the offset value in both the x and y direction.

    void ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true)
    {
        xoffset *= MouseSensitivity;
        yoffset *= MouseSensitivity;

        Yaw   += xoffset;
        Pitch += yoffset;

        // make sure that when pitch is out of bounds, screen doesn't get flipped
        if (constrainPitch)
        {
            if (Pitch > 89.0f)
                Pitch = 89.0f;
            if (Pitch < -89.0f)
                Pitch = -89.0f;
        }

        // update Front, Right and Up Vectors using the updated Euler angles
        UpdateCameraVectors();
    }

    // processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
    void ProcessMouseScroll(float yoffset)
    {
        Zoom -= (float)yoffset;
        if (Zoom < 1.0f)
            Zoom = 1.0f;
        if (Zoom > 45.0f)
            Zoom = 45.0f;
    }


private:

    void UpdateCameraVectors()
    {
        Eigen::Vector3f front;
        front[0] = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
        front[1] = sin(glm::radians(Pitch));
        front[2] = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
        Front = front.normalized();

        Right = Front.cross(WorldUp).normalized();
        Up = Right.cross(Front).normalized();
    }

};







#endif //IPC2DSIM_CAMERA_H
