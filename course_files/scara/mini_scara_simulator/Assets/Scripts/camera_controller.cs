using UnityEngine;

public class camera_controller : MonoBehaviour
{
    public float zoomSpeed = 5f;
    public float strafeSpeed = 5f;
    public float lookSpeed = 2f;
    public float minZoom = 0.1f;
    public float maxZoom = 0.2f;

    private float rotationX;
    private float rotationY;
    private bool isRotating = false;

    private Camera cam;

    void Start()
    {
        cam = GetComponent<Camera>();

        // Initialize rotation based on current rotation
        Vector3 euler = transform.rotation.eulerAngles;
        rotationX = euler.y;
        rotationY = euler.x;
    }

    void Update()
    {
        HandleZoom();
        HandleRotation();
        HandleMovement();
    }

    void HandleZoom()
    {
        if (Input.GetKey(KeyCode.W))
        {
            if (cam.orthographic)
                cam.orthographicSize = Mathf.Max(minZoom, cam.orthographicSize - zoomSpeed * Time.deltaTime);
            else
                cam.fieldOfView = Mathf.Max(minZoom * 10f, cam.fieldOfView - zoomSpeed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.S))
        {
            if (cam.orthographic)
                cam.orthographicSize = Mathf.Min(maxZoom, cam.orthographicSize + zoomSpeed * Time.deltaTime);
            else
                cam.fieldOfView = Mathf.Min(maxZoom, cam.fieldOfView + zoomSpeed * Time.deltaTime);
        }
    }

    void HandleMovement()
    {
        Vector3 move = Vector3.zero;

        if (Input.GetKey(KeyCode.A))
            move -= transform.right;
        if (Input.GetKey(KeyCode.D))
            move += transform.right;
        if (Input.GetKey(KeyCode.Space))
            move += transform.up;
        if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
            move -= transform.up;

        transform.position += move * strafeSpeed * Time.deltaTime;
    }

    void HandleRotation()
    {
        if (Input.GetMouseButtonDown(1))
        {
            isRotating = true;
        }
        if (Input.GetMouseButtonUp(1))
        {
            isRotating = false;
        }

        if (isRotating)
        {
            float mouseX = Input.GetAxis("Mouse X");
            float mouseY = Input.GetAxis("Mouse Y");

            rotationX += mouseX * lookSpeed;
            rotationY -= mouseY * lookSpeed;
            rotationY = Mathf.Clamp(rotationY, -90f, 90f);

            Quaternion targetRotation = Quaternion.Euler(rotationY, rotationX, 0f);
            transform.rotation = targetRotation;
        }
    }

    public void ResetCamera()
    {
        // Set the camera back to the original transform
        transform.position = new Vector3(1f, 0.725f, 1f);
        transform.rotation = Quaternion.Euler(25f, -135f, 0f);

        // Update internal rotation variables to match
        rotationX = -135f;
        rotationY = 25f;

        cam.orthographicSize = 0.2f;
    }
}
