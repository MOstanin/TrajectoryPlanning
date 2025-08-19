using UnityEngine;
using UnityEngine.InputSystem;

namespace TrajectoryPlanning.UI
{
    public class CameraController : MonoBehaviour
    {
        private Vector3 _currentRotation;
        private float _currentDistance;

        [SerializeField]
        private Vector3 pivotPoint;

        [Header("Rotation Settings")]
        [SerializeField]
        private float rotationSpeed = 200f;

        [Header("Zoom Settings")]
        [SerializeField]
        private float zoomSpeed = 10f;

        [SerializeField]
        private float minDistance = 2f;

        [SerializeField]
        private float maxDistance = 20f;

        private void Start()
        {
            _currentDistance = Vector3.Distance(transform.position, pivotPoint);
            var direction = (transform.position - pivotPoint).normalized;
            _currentRotation = Quaternion.LookRotation(direction).eulerAngles;
        }

        private void FixedUpdate()
        {
            HandleRotation();
            HandleZoom();
            UpdateCameraPosition();
        }

        private void HandleRotation()
        {
            var dt = Time.deltaTime;
            var mouse = Mouse.current;
            if (mouse != null && mouse.middleButton.isPressed)
            {
                var delta = mouse.delta.ReadValue();
                _currentRotation.y += delta.x * rotationSpeed * dt * 0.1f;
                _currentRotation.x -= delta.y * rotationSpeed * dt * 0.1f;
            }

            _currentRotation.x = Mathf.Clamp(_currentRotation.x, -89f, 89f);
        }

        private void HandleZoom()
        {
            var zoomDelta = 0f;

            var mouse = Mouse.current;
            if (mouse != null)
            {
                var scrollY = mouse.scroll.ReadValue().y;
                if (Mathf.Abs(scrollY) > 0.01f)
                {
                    zoomDelta -= scrollY * (zoomSpeed * 0.01f);
                }
            }

            if (Mathf.Abs(zoomDelta) > Mathf.Epsilon)
            {
                _currentDistance += zoomDelta;
                _currentDistance = Mathf.Clamp(_currentDistance, minDistance, maxDistance);
            }
        }

        private void UpdateCameraPosition()
        {
            var rotation = Quaternion.Euler(_currentRotation);
            var direction = rotation * Vector3.forward;
            transform.position = pivotPoint - direction * _currentDistance;
            transform.LookAt(pivotPoint);
        }
    }
}
