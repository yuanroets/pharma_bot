# GUI_Report: nav_gui Web App Integration and Technical Overview

## Introduction
This report details the design, implementation, and rationale behind the GUI-based navigation system for the pharma_bot project. The system leverages a web application (webhook) to provide universal, user-friendly access to robot navigation, interfacing with ROS2 via a FastAPI server and custom ROS2 node (`nav_gui`).

## System Architecture and Workflow
The navigation GUI is a web app developed using a template provided by a collaborator. This app acts as a webhook, allowing users to send navigation requests from any device with a browser. The workflow is as follows:

1. **Web App (Webhook) Interface:**
   - Users access the web app via a public URL (exposed using ngrok).
   - The app presents a simple interface for entering target coordinates (x, y, yaw/radians) and sending navigation requests.
   - The web app is device-agnostic; any staff member in the hospital can use it from their phone, tablet, or computer.

2. **FastAPI Server:**
   - The web app sends HTTP POST requests to a FastAPI server running on the robot or a local machine.
   - The FastAPI server exposes a `/navigate` endpoint, which receives the coordinates and orientation from the web app.
   - CORS middleware is enabled to allow requests from any origin, ensuring compatibility with the web app.

3. **ROS2 Integration via nav_gui Node:**
   - The FastAPI server publishes a ROS2 `PoseStamped` message to the `/nav_goal` topic.
   - The custom `nav_gui` ROS2 node subscribes to `/nav_goal` and extracts the navigation goal.
   - The node then sends the goal to the Nav2 stack using the `NavigateToPose` action client, triggering autonomous navigation.

4. **Endpoint Exposure and Universal Access:**
   - The FastAPI server is made publicly accessible using ngrok, which tunnels the local port to a secure public URL.
   - This approach allows anyone with the URL to send navigation requests, making the system highly flexible and scalable for hospital use.

## Technical Rationale and Advantages
- **Universal Accessibility:** By using a web app and exposing the endpoint via ngrok, the system is not tied to any specific device or operating system. Any authorized user can access the robot from anywhere within the hospital network.
- **Separation of Concerns:** The architecture cleanly separates the user interface (web app), API server (FastAPI), and robot control (ROS2 node). This modularity simplifies maintenance and future upgrades.
- **Security and Flexibility:** ngrok provides secure tunneling, and FastAPI allows for easy integration of authentication or access control if needed.
- **Ease of Use:** The web app’s intuitive interface lowers the barrier for non-technical users to operate the robot, supporting real-world deployment in a hospital environment.
- **Scalability:** The webhook/API model supports multiple simultaneous users and can be extended to other robots or services as needed.

## Implementation Details
- **Web App:** Developed using a template, customized for pharma_bot navigation. Accepts coordinates and orientation in the map frame.
- **FastAPI Server:** Python-based, runs alongside ROS2. Handles incoming requests, converts payloads, and publishes to ROS2 topics.
- **nav_gui Node:** ROS2 Python node, subscribes to `/nav_goal`, interfaces with Nav2, and manages navigation actions.
- **ngrok:** Used to expose the FastAPI server to the public internet, providing a secure, temporary URL for the web app.

## Setup and Usage
1. Start ROS2 and launch the `nav_gui` node.
2. Start the FastAPI server (`python3 nav_api.py`).
3. Run ngrok to expose the server (`ngrok http 8000`).
4. Share the ngrok URL with hospital staff; they can access the web app and send navigation requests.

## Conclusion
This web-based navigation system for pharma_bot provides a robust, universal, and user-friendly solution for hospital environments. By leveraging modern web technologies, FastAPI, and ROS2, the system enables seamless integration between user requests and robot navigation, supporting real-world deployment and future scalability. The modular design ensures maintainability and adaptability, while universal access empowers staff to interact with the robot from any device, enhancing operational efficiency and patient care.
