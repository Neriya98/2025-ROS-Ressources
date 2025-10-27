#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime


class ImageRecorderNode(Node):
    def __init__(self):
        super().__init__('image_recorder_node')
        
        # Créer le répertoire de stockage
        home_dir = os.path.expanduser('~')
        self.photos_dir = os.path.join(home_dir, 'photos')
        os.makedirs(self.photos_dir, exist_ok=True)
        self.get_logger().info(f'Stockage des images dans: {self.photos_dir}')
        
        # Subscriber pour la caméra
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Bridge pour convertir entre ROS et OpenCV
        self.bridge = CvBridge()
        
        # Compteur d'images
        self.image_count = 0
        self.last_save_time = 0.0
        self.save_interval = 1.0  # Sauvegarder une image toutes les 1 seconde
        
        self.get_logger().info('Image Recorder Node started')

    def image_callback(self, msg):
        try:
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # Limiter la fréquence de sauvegarde
            if current_time - self.last_save_time < self.save_interval:
                return
            
            self.last_save_time = current_time
            
            # Convertir l'image ROS en OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Générer un nom de fichier avec timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = os.path.join(self.photos_dir, f'photo_{timestamp}.png')
            
            # Sauvegarder en PNG (sans compression de perte)
            success = cv2.imwrite(filename, cv_image)
            
            if success:
                self.image_count += 1
                if self.image_count % 10 == 0:
                    self.get_logger().info(f'Sauvegardées {self.image_count} images - Dernière: {filename}')
            else:
                self.get_logger().error(f'Erreur lors de la sauvegarde de {filename}')
                
        except Exception as e:
            self.get_logger().error(f'Erreur lors du traitement de l\'image: {e}')


def main(args=None):
    rclpy.init(args=args)
    recorder = ImageRecorderNode()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
