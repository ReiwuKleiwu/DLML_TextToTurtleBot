#!/usr/bin/env python3
"""
Demonstration script for object position estimation using camera and LIDAR fusion.

This script shows how to use the enhanced TextToTurtlebotNode with object position estimation
capabilities. It demonstrates:
1. Starting the robot to explore and detect objects
2. Retrieving detected object positions from the map
3. Getting statistics about detections
4. Finding nearest objects to the robot

Usage:
    python3 demo_object_position_estimation.py
"""

import rclpy
import time
from classes.nodes.TextToTurtlebotNode import TextToTurtlebotNode
from classes.controllers.MovementThread import MovementThread


def main():
    rclpy.init()
    
    # Create the enhanced TurtleBot node with position estimation
    print("Initializing TextToTurtleBot with object position estimation...")
    turtlebot_node = TextToTurtlebotNode(namespace="", use_turtlebot_sim=True)
    
    # Start looking for a specific object (you can change this to any object from the YOLO model)
    target_object = "chair"
    print(f"Starting exploration to find: {target_object}")
    turtlebot_node.find_target(target_object)
    
    # Start movement thread
    movement_thread = MovementThread(2, turtlebot_node)
    
    try:
        print("Robot is now exploring and detecting objects...")
        print("Object positions will be estimated using camera and LIDAR fusion.")
        print("Press Ctrl+C to stop and show results.")
        
        # Let the robot run for a while to collect data
        start_time = time.time()
        
        while rclpy.ok():
            # Spin once to process callbacks
            rclpy.spin_once(turtlebot_node, timeout_sec=0.1)
            
            # Every 10 seconds, show some statistics
            if time.time() - start_time > 10.0:
                print("\n" + "="*50)
                print("OBJECT DETECTION STATISTICS")
                print("="*50)
                
                # Get detection statistics
                stats = turtlebot_node.get_detection_statistics()
                if stats:
                    for obj_class, stat in stats.items():
                        print(f"{obj_class}: {stat['count']} detections, "
                              f"avg confidence: {stat['avg_confidence']:.2f}")
                else:
                    print("No objects detected yet.")
                
                # Show nearest objects to robot
                print("\nNEAREST OBJECTS:")
                robot_position = turtlebot_node.tf_subscriber.get_position_2d()
                if robot_position:
                    print(f"Robot position: ({robot_position[0]:.2f}, {robot_position[1]:.2f})")
                    
                    # Get objects within 5 meters
                    nearby_objects = turtlebot_node.get_objects_in_radius(5.0)
                    if nearby_objects:
                        for obj in nearby_objects[:5]:  # Show first 5
                            pos = obj['position']
                            print(f"  {obj['class']} at ({pos[0]:.2f}, {pos[1]:.2f}) "
                                  f"- distance: {obj['distance']:.2f}m, confidence: {obj['confidence']:.2f}")
                    else:
                        print("  No objects detected within 5 meters.")
                else:
                    print("Robot position not available yet.")
                
                # Clean up old detections (older than 5 minutes)
                turtlebot_node.cleanup_old_detections(300.0)
                
                start_time = time.time()
                print("="*50 + "\n")
    
    except KeyboardInterrupt:
        print("\nStopping robot...")
        
        # Final statistics
        print("\n" + "="*50)
        print("FINAL DETECTION SUMMARY")
        print("="*50)
        
        all_objects = turtlebot_node.get_detected_objects()
        total_detections = sum(len(objects) for objects in all_objects.values())
        print(f"Total detections: {total_detections}")
        
        stats = turtlebot_node.get_detection_statistics()
        for obj_class, stat in stats.items():
            print(f"\n{obj_class}:")
            print(f"  Total detections: {stat['count']}")
            print(f"  Average confidence: {stat['avg_confidence']:.2f}")
            print(f"  Latest detection: {time.ctime(stat['latest_detection'])}")
            
            # Show a few example positions for this object class
            objects = turtlebot_node.get_detected_objects(obj_class)
            print(f"  Recent positions:")
            for i, obj in enumerate(objects[-3:]):  # Last 3 detections
                pos = obj['position']
                print(f"    {i+1}. ({pos[0]:.2f}, {pos[1]:.2f}) "
                      f"confidence: {obj['confidence']:.2f}")
        
        print("="*50)
    
    finally:
        movement_thread.end()
        turtlebot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
