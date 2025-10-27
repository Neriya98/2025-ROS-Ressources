#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory


class ImageCapturerNode(Node):
    def __init__(self):
        super().__init__('image_capturer_node')
        
        # Dossier de destination - dans le package qr_code_scanner
        try:
            pkg_share = get_package_share_directory('qr_code_scanner')
            self.photos_dir = os.path.join(pkg_share, '..', '..', 'src', '2025-ROS-Ressources', 'qr_code_scanner', 'results', 'images')
            self.photos_dir = os.path.abspath(self.photos_dir)
        except Exception as e:
            # Fallback si le package n'est pas trouvé
            self.photos_dir = os.path.expanduser('~/qr_images')
            self.get_logger().warn(f'Fallback directory: {self.photos_dir}, erreur: {e}')
        
        # Créer le dossier s'il n'existe pas
        os.makedirs(self.photos_dir, exist_ok=True)
        self.get_logger().info(f'Sauvegarde des photos dans: {self.photos_dir}')
        
        # Subscriber pour l'image de la caméra
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Bridge pour convertir entre ROS et OpenCV
        self.bridge = CvBridge()
        
        # Counter d'images
        self.image_count = 0

        self.get_logger().info('Image Capturer Node started')

    def image_callback(self, msg):
        try:
            self.image_count += 1
            
            # Convertir l'image ROS en OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Upscaler x2 pour meilleure résolution avec meilleure interpolation
            h, w = cv_image.shape[:2]
            upscaled = cv2.resize(cv_image, (w * 2, h * 2), interpolation=cv2.INTER_LANCZOS4)
            
            # Générer le nom du fichier avec timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # milliseconds
            filename = f"camera_{timestamp}.png"
            filepath = os.path.join(self.photos_dir, filename)
            
            # Sauvegarder l'image en PNG haute qualité (compression maximale)
            cv2.imwrite(filepath, upscaled, [cv2.IMWRITE_PNG_COMPRESSION, 9])
            
            if self.image_count % 10 == 0:
                self.get_logger().info(f'Photos capturées: {self.image_count} (dernier: {filename} - {upscaled.shape[1]}x{upscaled.shape[0]})')
            
        except Exception as e:
            self.get_logger().error(f'Erreur lors de la capture: {e}')


def main(args=None):
    rclpy.init(args=args)
    capturer = ImageCapturerNode()
    try:
        rclpy.spin(capturer)
    except KeyboardInterrupt:
        pass
    finally:
        capturer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
