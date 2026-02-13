# **caatinga\_vision**

A ROS 2 computer vision pipeline developed for smart agricultural robotics, specifically designed for traceability and object detection in the field. This package utilises the **YOLOv4-tiny** architecture to provide real-time inference on resource-constrained hardware.

## **Overview**

The `caatinga_vision` package serves as the primary visual perception layer for the Caatinga Robotics platform. It processes raw camera feeds to detect and track objects—ranging from agricultural equipment to people and livestock—facilitating autonomous navigation and crop traceability.

### What the Robot "Sees"

The system is currently configured with the COCO Dataset, enabling it to recognise 80 common object classes out of the box. For an agricultural robot, this provides immediate functional capabilities:
- Safety & Obstacle Avoidance: Real-time detection of person, dog, and truck ensures the robot can stop or navigate around farmworkers and domestic animals.
- Livestock Traceability: Built-in support for cow, sheep, and horse allows for automated headcounts and fence-line monitoring.
- Operational Awareness: Identification of backpacks, bottles, or tools helps in maintaining a clean and safe working environment in the field.

### Why YOLO for Agriculture?
- Traceability: By converting video pixels into structured data (Detection2DArray), the robot logs the "Who, What, and Where" of every encounter. This creates a digital audit trail essential for modern smart farming.
- Edge Efficiency: Unlike standard AI that requires powerful cloud servers, YOLOv4-tiny is optimised for "The Edge." It runs directly on the robot's onboard computer, ensuring 24/7 operation even in remote fields with no internet connection.
- Low Latency: The "You Only Look Once" approach processes the entire frame in a single mathematical pass. This allows the robot to react to a moving obstacle (like a worker stepping into its path) in milliseconds.

### Future Scalability: Transfer Learning

The caatinga_vision package is designed as a base layer. By replacing the coco.names and .weights files, the pipeline can be "re-trained" to detect specific agricultural interests such as:
- Identifying weeds vs. crops for precision spraying.
- Detecting fruit ripeness levels for automated harvesting.
- Monitoring plant stress or disease markers.


### **Key Features**

* **ROS 2 Integration:** Fully compatible with ROS 2 (Humble/Foxy) using `rclpy`.  
* **YOLOv4-tiny Inference:** High-speed object detection using a lightweight neural network.  
* **Modular Config:** Easy-to-adjust parameters for different environments (e.g., simulation vs. real-world deployment).  
* **Agro-Focused:** Specifically tuned for the requirements of smart agricultural traceability.

## **Package Structure**
```
caatinga_vision/
├── config/             # Parameter files (.yaml) and class labels (.names)
├── launch/             # ROS 2 launch scripts
├── models/             # YOLO neural network configuration and weights
├── caatinga_vision/    # Python source code
│   ├── __init__.py
│   ├── caatinga_vision_node.py  # Main ROS 2 node
│   └── tools.py                 # Image processing utilities
├── package.xml         # Package metadata and dependencies
└── setup.py            # Build and installation script
```

## **Prerequisites**

Ensure you have the following installed on your system:

* **ROS 2** (Humble Hawksbill recommended)  
* **OpenCV** (`python3-opencv`)  
* **cv\_bridge** (`ros-humble-cv-bridge`)  
* **vision\_msgs** (`ros-humble-vision-msgs`)


### **Topics**

* **Subscribes to:** `image_raw` (`sensor_msgs/Image`)  
* **Publishes to:** `detections` (`vision_msgs/Detection2DArray`)

## **Technical Details**

### **Model Weights**

The package uses **YOLOv4-tiny**, which provides a significant performance boost over the standard YOLOv4 model at the cost of a slight reduction in accuracy. This is ideal for mobile robots where CPU/GPU resources and battery life are critical factors.

### **Pre-processing**

Images are automatically resized to **416x416** pixels and normalised (values 0–1) before being passed to the neural network, as defined in `tools.py`.

## **Maintainers**

* **João de Moura** \- joaodemouragy@gmail.com

## **Licence**

This project is licensed under the **Apache-2.0 Licence** \- see the `package.xml` for details.  
