# Python-Programm, das den TurtleBot dazu befähigt, den Raum autonom zu erkunden, Hindernissen (basierend auf LiDAR
# und Infrarotsensoren) auszuweichen, dabei Objekte zu detektieren, den Raum zu klassifizieren und durch eine Tür zu
# fahren, wenn diese erkannt wird. Die Liste der detektierten Objekte wird nach dem Durchfahren der Tür gelöscht.

# Erklärung der neuen Funktionen:
# Hindernisvermeidung (LiDAR): Das Programm verwendet die LiDAR-Daten, um Hindernisse zu erkennen. Wenn der TurtleBot
# einem Hindernis zu nahe kommt (weniger als 0.5 m), stoppt er und dreht sich zufällig, um den Weg freizumachen.
#
# Zufällige Erkundung: Wenn keine Hindernisse vorhanden sind, bewegt sich der TurtleBot zufällig vorwärts, indem er
# sich leicht dreht und geradeaus fährt.
#
# Objektdetektion während der Erkundung: Der TurtleBot führt kontinuierlich Objektdetektionen mit der OAK-D Pro
# Kamera durch. Sobald 5 verschiedene Objekte erkannt wurden, wird ein LLM verwendet, um basierend auf den detektierten
# Objekten den Raumtyp zu bestimmen.
#
# Türerkennung und Durchfahren: Wenn eine Tür erkannt wird, fährt der TurtleBot durch sie hindurch. Nachdem er durch
# die Tür gefahren ist, wird die Liste der detektierten Objekte gelöscht, und die Klassifizierung des neuen Raums
# beginnt von vorne.
#
# Schritte zur Nutzung:
# Stelle sicher, dass die OAK-D Pro Kamera und LiDAR auf dem TurtleBot korrekt konfiguriert sind.
# Die erforderlichen Abhängigkeiten (z.B. torch, ultralytics, transformers, cv_bridge) sollten installiert sein.
# Das Programm startet als ROS-Node und abonniert die relevanten Topics (/camera/color/image_raw für die Kamera und
# /scan für LiDAR).
# Installation der Abhängigkeiten:

# pip install torch torchvision ultralytics transformers

# Dieses Programm bietet dir eine vollständige Lösung zur autonomen Erkundung und Klassifizierung von Räumen mithilfe
# von YOLO für Objektdetektion und Huggingface LLM für die Klassifizierung.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import torch
from transformers import AutoTokenizer, AutoModelForCausalLM, pipeline
import random
from irobot_create_msgs.msg import IrIntensityVector, IrIntensity
from enum import Enum
import os
import time
import math
import threading

from ultralytics import YOLO

import ollama


class ExploreAndDetectNode(Node):
    IR_DISTANCE_MIN = 175  # smaller = bigger dist; bigger = later object detection
    SEARCHED_OBJECT_CLASS = "bin"

    class State(Enum):
        EXPLORE = 0  # Raum wird mit Movement-Logik abgesucht
        AVOID_OBSTACLE = 1  # Hindernis wird umgangen
        OBJECT_FOUND = 2  # Objekt wird gesehen und muss nun angesteuert werden
        OBJECT_REACHED = 3  # Objet wurde erfolgreich erreicht
        EXIT = 4

    class StateSource(Enum):
        GOAL = 0
        LIDAR = 1
        IR = 2
        BUMPER = 3
        CAMERA = 4

    def __init__(self):
        super().__init__('explore_and_detect_node')

        self.state = [(ExploreAndDetectNode.State.EXPLORE, ExploreAndDetectNode.StateSource.GOAL, None)]

        # Kamera-Abonnement und LiDAR
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/robot_1/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/robot_1/scan', self.lidar_callback, 10)

        # self.bumper_sub = self.create_subscription(BumperMsgType, '/bumper', self.bumper_callback, 10)

        # Abonniere die Infrarotsensoren (IR-Intensität)
        self.ir_subscriber = self.create_subscription(
            IrIntensityVector,
            "/robot_1/ir_intensity",
            self.ir_callback,
            qos_profile_sensor_data
        )

        # Publikator für Bewegungen (Twist-Nachricht)
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot_1/cmd_vel', 10)

        # Initialisierung von Bewegungssteuerung
        self.twist = Twist()
        self.lidar_data = []

        # Detektierte Objekte und Raumklassifizierung
        self.detected_objects = set()
        self.detected_objects_dict = {}
        self.room_classified = False
        self.door_detected = False
        self.passed_through_door = False

    def movement(self):
        match self.state[-1][0]:
            case ExploreAndDetectNode.State.EXPLORE:
                self.get_logger().info("Exploring...")
                self.random_exploration()
            case ExploreAndDetectNode.State.AVOID_OBSTACLE:
                self.get_logger().info("Avoiding Obstacle...")
                self.avoid_obstacle()
            case ExploreAndDetectNode.State.OBJECT_FOUND:
                print("Object Found!")
                self.navigate_to_detected_object()
            case ExploreAndDetectNode.State.OBJECT_REACHED:
                print("Object Reached!")
            case _:
                print("Unknown state!")

    #done
    def change_state(self, state, source, parameters=None):
        # Check if most recent state matches new state
        if (self.state[-1][0] == state): return

        # Check if source does not match most recent source
        # if(self.state[-1][1] != source): return

        self.get_logger().info(f"New state: {state} | Source: {source}")

        self.state.append((state, source, parameters))

    # done
    def pop_state(self, state, source):
        if (self.state[-1][0] != state): return
        if (self.state[-1][1] != source): return

        self.get_logger().info(f"Popped state: {state} | Source: {source}")

        self.state.pop()

    # TODO
    def lidar_callback(self, msg):
        # Extrahieren der Winkelinformationen aus dem LaserScan
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Berechnung des Index für die Front des Roboters
        index_front = round((-1.5708 - angle_min) / angle_increment)  # -1.5708 rad entspricht -90 Grad

        # Überprüfen, ob ein Hindernis im 50-Grad-Bereich vor dem Roboter ist
        index_start = max(0, index_front - 50)
        index_end = min(len(msg.ranges) - 1, index_front + 50)
        front_ranges = msg.ranges[index_start:index_end + 1]
        front_distance = min([x for x in front_ranges if x != float('inf')])

        self.get_logger().info("LIDAR: " + str(front_distance))

        # Hindernisvermeidung
        if front_distance < 0.5:  # Wenn ein Hindernis näher als 0.5m ist
            self.get_logger().info("Vermeide Hindernis weil näher als 0.5m")
            # self.avoid_obstacle()
            direction = random.choice([-1, 1])
            self.change_state(ExploreAndDetectNode.State.AVOID_OBSTACLE, ExploreAndDetectNode.StateSource.LIDAR,
                              direction)

        else:
            self.pop_state(ExploreAndDetectNode.State.AVOID_OBSTACLE, ExploreAndDetectNode.StateSource.LIDAR)
            # self.random_exploration()

    # TODO
    def bumper_callback(self, msg):
        # Beispielhafter Zugang zu den Bumper-Daten (abhängig vom TurtleBot-Typ)
        if msg.state == "TRIGGERED":  # Wenn der Bumper ein Hindernis meldet
            self.get_logger().info("Vermeide Hindernis weil Bumper Kontakt meldet")
            # self.avoid_obstacle()
            self.change_state(ExploreAndDetectNode.State.AVOID_OBSTACLE, ExploreAndDetectNode.StateSource.BUMPER)
        else:
            self.pop_state(ExploreAndDetectNode.State.AVOID_OBSTACLE, ExploreAndDetectNode.StateSource.BUMPER)

    # TODO
    def ir_callback(self, ir_intensity_vector: IrIntensityVector):
        # Die Intensität der Infrarotsensoren auswerten
        # Angenommen, der Sensorwert zeigt den Abstand zu einem Hindernis (z.B. je größer die Intensität, desto näher
        # das Objekt)
        # if an object too close, just turn until nothing is in front of robo anymore
        # ir and laser min dist check
        if any(dist.value > self.IR_DISTANCE_MIN for dist in ir_intensity_vector.readings):
            self.get_logger().info("Vermeide Hindernis weil IR Sensor Hindernis meldet")

            self.change_state(ExploreAndDetectNode.State.AVOID_OBSTACLE, ExploreAndDetectNode.StateSource.IR)
            # self.avoid_obstacle()
        else:
            self.pop_state(ExploreAndDetectNode.State.AVOID_OBSTACLE, ExploreAndDetectNode.StateSource.IR)

    # done
    def image_callback(self, msg):
        # Bild von der Kamera empfangen
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # YOLO-Detektion auf dem Bild durchführen
        results = yolo_model.predict(cv_image, max_det=20, verbose=False)  # device="cuda:0",

        boxes = results[0].boxes

        for icol, box in enumerate(boxes):
            cls = int(boxes.cls[icol])
            classname = results[0].names[cls]
            #
            # Speichere die gefundenen Objekte
            self.detected_objects.add(classname)

            # Zeichne Bounding Box auf dem Bild
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Koordinaten der Bounding Box

            self.detected_objects_dict[classname] = {
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
            }

            color = (0, 0, 255) if (classname == self.SEARCHED_OBJECT_CLASS) else (0, 255, 0)

            cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)  # Grüne Bounding Box
            cv2.putText(cv_image, classname, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        self.detect_target_object()

        if len(self.detected_objects) > 0:
            cv2.putText(cv_image, ', '.join(self.detected_objects), (5, 460 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 255), 2)
        if self.room_classified:
            cv2.putText(cv_image, self.room_type, (5, 460 - 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 255), 2)

        # Schaue nach ob gesuchtes Objekt in gefundenen Objekten

        # Zeige das Bild im Fenster
        cv2.imshow('TurtleBot Camera', cv_image)
        cv2.waitKey(1)  # Erforderlich, um das Fenster zu aktualisieren

        # Prüfen, ob mindestens 3 verschiedene Objekte detektiert wurden
        if len(self.detected_objects) >= 3 and not self.room_classified:
            self.classify_room()

        # Erkennen der Tür und Handeln, wenn eine Tür detektiert wurde
        if 'open door' in self.detected_objects and not self.door_detected:
            self.door_detected = True
            # self.get_logger().info("Tür detektiert, fahre hindurch...")
            self.pass_through_door()

    # done
    def detect_target_object(self):
        if (self.SEARCHED_OBJECT_CLASS not in self.detected_objects): return
        self.change_state(ExploreAndDetectNode.State.OBJECT_FOUND, ExploreAndDetectNode.StateSource.CAMERA)

    # TODO
    def map_to_minus1_to_1(self, x, a, b):
        return 2 * (x - a) / (b - a) - 1

    # TODO
    def navigate_to_detected_object(self):
        detected_object = self.detected_objects_dict[self.SEARCHED_OBJECT_CLASS]
        print(detected_object)

        width = detected_object["x2"] - detected_object["x1"]
        center = detected_object["x1"] + (width / 2)

        camera_center = 125

        direction = center - camera_center

        print(f"Direction: {direction}")
        if (abs(direction) >= 25):
            turn_direction = self.map_to_minus1_to_1(direction, -125, 125)
            self.twist.linear.x = 0.0  # Stoppen
            self.twist.angular.z = -0.1 * turn_direction
        else:
            self.twist.linear.x = 0.2

        self.cmd_vel_pub.publish(self.twist)

    # done
    def random_exploration(self):
        # Zufällige Bewegungssteuerung für die Erkundung
        self.twist.linear.x = 0.2  # Vorwärtsbewegung
        self.twist.angular.z = 0.0  # gerade aus fahren
        self.cmd_vel_pub.publish(self.twist)

    # done
    def avoid_obstacle(self):
        # Wenn ein Hindernis erkannt wurde, stoppe und drehe zufällig
        self.twist.linear.x = 0.0  # Stoppen

        self.twist.angular.z = 0.4 * self.state[-1][2]
        self.cmd_vel_pub.publish(self.twist)

    def classify_room(self):
        objects_list = ', '.join(self.detected_objects)

        # Erstelle den Prompt für das LLM
        prompt = (
            f"I am a robot navigating through a university building. I can see the following objects with my camera: {objects_list}. "
            f"Based on these objects, my task is to determine the room type. "
            f"Please respond only with the type of room (e.g., 'kitchen', 'classroom', 'bathroom', 'laboratory', "
            f"'meeting room', 'hallway', etc.) and nothing else."
        )

        try:
            # Call an das lokale Ollama-Modell
            response = ollama.chat(
                model='llama3',  # oder 'mistral', 'gemma' usw. – je nachdem, was du lokal geladen hast
                messages=[
                    {'role': 'user', 'content': prompt}
                ]
            )

            self.room_type = response['message']['content'].strip()

        except Exception as e:
            self.get_logger().warn(f"Ollama-Fehler: {e}")
            self.room_type = "unknown"

        print("******** " + self.room_type + " ********")
        self.get_logger().info(f"Raumtyp erkannt: {self.room_type}")
        self.room_classified = True

    def pass_through_door(self):
        # Durch die Tür fahren (vorwärts)
        self.twist.linear.x = 0.3
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

        # Warten für eine Weile (um anzunehmen, dass der Turtlebot durch die Tür gefahren ist)
        self.get_logger().info("Fahre durch die Tür...")
        time.sleep(3)

        # Nachdem die Tür passiert wurde, Liste zurücksetzen
        self.detected_objects.clear()
        self.room_classified = False  # Damit der neue Raum klassifiziert werden kann
        self.door_detected = False

        self.get_logger().info("Raum gewechselt, Objektdaten gelöscht.")

# done
def create_custom_yolo_model():
    if True:
        model = YOLO("yolov8s-worldv2.pt")  # Initialize a YOLO-World model
        model.set_classes(["table", "monitor", "closed door", "open door", "chair", "computer",
                           "person", "fridge", "fire extinguisher", "window", "blackboard",
                           "kitchen cabinet", "wall", "toilet", "towel", "radiator", "desk",
                           "bin"])  # Define custom classes
        model.save("custom_yolov8s.pt")  # Save the model with the defined offline vocabulary

    model = YOLO("custom_yolov8s.pt")  # Load your custom model

    return model

# TODO
class MovementThread(threading.Thread):
    def __init__(self, rate, node):
        super(MovementThread, self).__init__()
        self.condition = threading.Condition()
        self.done = False
        self.node = node
        # self.condition = threading.Condition()

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def run(self):
        while not self.done:
            self.node.movement()
            self.condition.acquire()
            self.condition.wait(self.timeout)
            self.condition.release()

    def stop(self):
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = ExploreAndDetectNode()

    try:
        movement_thread = MovementThread(2, node)

        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    finally:
        movement_thread.end()


if __name__ == '__main__':
    yolo_model = create_custom_yolo_model()

    main()