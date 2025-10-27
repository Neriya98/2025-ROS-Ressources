#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge
from pyzbar import pyzbar
import cv2
import numpy as np
import math
import json
import os
from ament_index_python.packages import get_package_share_directory


class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector_node')
        
        # Chemin du fichier JSON pour stocker les données QR
        try:
            pkg_share = get_package_share_directory('qr_code_scanner')
            results_dir = os.path.join(pkg_share, '..', '..', 'src', '2025-ROS-Ressources', 'qr_code_scanner', 'results', 'qr_codes')
            results_dir = os.path.abspath(results_dir)
        except Exception as e:
            # Fallback si le package n'est pas trouvé
            results_dir = os.path.expanduser('~/qr_codes_results')
            self.get_logger().warn(f'Fallback directory: {results_dir}, erreur: {e}')
        
        os.makedirs(results_dir, exist_ok=True)
        self.data_file = os.path.join(results_dir, 'qr_data.json')
        self.load_existing_data()
        self.get_logger().info(f'Stockage des données QR dans: {self.data_file}')
        
        # Créer un subscriber pour l'image de la caméra
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Topic de la caméra du X3
            self.image_callback,
            10)
        
        # Publisher pour les données QR code
        self.qr_pub = self.create_publisher(String, 'detected_qr_codes', 10)
        
        # Bridge pour convertir entre ROS et OpenCV
        self.bridge = CvBridge()
        
        # Subscriber LIDAR
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        
        # Subscriber odométrie
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Variables pour stocker les informations contextuelles
        self.last_pose = None
        self.last_lidar_distance = None
        self.detected_qr_codes = {}
        
        # Counter images
        self.image_count = 0
        
        # Cache pour éviter les duplicatas
        self.qr_last_seen = {}
        self.qr_cache_timeout = 3.0  # secondes
        
        # Pour l'affichage GUI
        self.display_enabled = True

        self.get_logger().info('QR Detector Node started')

    def load_existing_data(self):
        """Charger les données QR existantes depuis le fichier JSON"""
        try:
            if os.path.exists(self.data_file):
                with open(self.data_file, 'r') as f:
                    self.detected_qr_codes = json.load(f)
        except Exception as e:
            self.get_logger().warn(f'Erreur lors du chargement des données: {e}')
            self.detected_qr_codes = {}

    def save_qr_data(self, qr_content):
        """Sauvegarder les données QR dans le fichier JSON"""
        try:
            # Créer une entrée avec timestamp et informations contextuelles
            qr_entry = {
                'content': qr_content,
                'timestamp': self.get_clock().now().to_msg().sec,
                'pose': self.last_pose,
                'lidar_distance': self.last_lidar_distance
            }
            
            # Ajouter ou mettre à jour l'entrée
            if qr_content not in self.detected_qr_codes:
                self.detected_qr_codes[qr_content] = []
            
            self.detected_qr_codes[qr_content].append(qr_entry)
            
            # Sauvegarder dans le fichier
            with open(self.data_file, 'w') as f:
                json.dump(self.detected_qr_codes, f, indent=2)
                
        except Exception as e:
            self.get_logger().error(f'Erreur lors de la sauvegarde: {e}')

    def image_callback(self, msg):
        try:
            self.image_count += 1
            if self.image_count % 30 == 0:
                self.get_logger().info(f'Images reçues: {self.image_count}')
            
            # Convertir l'image ROS en image OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # DEBUG: Logger la forme et le format de l'image
            if self.image_count % 100 == 0:
                self.get_logger().info(f'Image shape: {cv_image.shape}, dtype: {cv_image.dtype}')
            
            # Prétraitement pour meilleure détection
            # 1. Convertir en niveaux de gris (pyzbar fonctionne mieux)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # 2. Améliorer le contraste avec CLAHE (Contrast Limited Adaptive Histogram Equalization)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)
            
            # 3. Appliquer un threshold adaptatif pour améliorer la détection
            _, binary = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            
            # Détecter les QR codes
            qr_codes = None
            detection_attempt = 1
            
            # Essayer d'abord sur l'image binaire
            try:
                qr_codes = pyzbar.decode(binary)
                if len(qr_codes) > 0:
                    self.get_logger().info(f'✓ {len(qr_codes)} QR code(s) détecté(s) (binary image)')
                    detection_attempt = 1
            except Exception as decode_err:
                self.get_logger().error(f'Erreur pyzbar sur binary: {decode_err}')
                qr_codes = []
            
            # Si aucune détection, essayer sur l'image en niveaux de gris
            if len(qr_codes) == 0:
                try:
                    qr_codes = pyzbar.decode(enhanced)
                    if len(qr_codes) > 0:
                        self.get_logger().info(f'✓ {len(qr_codes)} QR code(s) détecté(s) (enhanced grayscale)')
                        detection_attempt = 2
                except Exception as decode_err:
                    self.get_logger().error(f'Erreur pyzbar sur enhanced: {decode_err}')
                    qr_codes = []
            
            # Si toujours aucune détection, essayer sur l'image originale
            if len(qr_codes) == 0:
                try:
                    qr_codes = pyzbar.decode(cv_image)
                    if len(qr_codes) > 0:
                        self.get_logger().info(f'✓ {len(qr_codes)} QR code(s) détecté(s) (original image)')
                        detection_attempt = 3
                except Exception as decode_err:
                    self.get_logger().error(f'Erreur pyzbar sur original: {decode_err}')
                    qr_codes = []
            
            if self.image_count % 100 == 0:
                self.get_logger().info(f'pyzbar.decode() appelé (tentative {detection_attempt}), résultat: {len(qr_codes)} QR détectés')
            
            import time
            current_time = time.time()
            
            for qr in qr_codes:
                try:
                    # Extraire le contenu du QR code
                    qr_data = qr.data.decode('utf-8')
                except Exception as e:
                    self.get_logger().warn(f'Erreur décodage QR: {e}')
                    continue
                
                # Vérifier le cache pour éviter les duplicatas
                if qr_data in self.qr_last_seen:
                    if current_time - self.qr_last_seen[qr_data] < self.qr_cache_timeout:
                        continue
                
                self.qr_last_seen[qr_data] = current_time
                
                # Publier le QR code détecté
                qr_msg = String()
                qr_msg.data = qr_data
                self.qr_pub.publish(qr_msg)
                
                # Log des informations
                pose_str = ""
                if self.last_pose:
                    pose_str = f", Position robot: x={self.last_pose[0]:.2f}, y={self.last_pose[1]:.2f}, orientation={self.last_pose[2]:.2f} rad"
                
                lidar_str = ""
                if self.last_lidar_distance:
                    lidar_str = f", Distance LIDAR devant le robot: {self.last_lidar_distance:.2f} m"
                
                self.get_logger().info(f'✓✓ Code QR PUBLIÉ: {qr_data}{pose_str}{lidar_str}')
                
                # Sauvegarder les données
                self.save_qr_data(qr_data)
            
            # Afficher l'image en direct (avec gestion d'erreur pour X11)
            if self.display_enabled:
                try:
                    # Afficher l'image avec annotations
                    display_img = cv_image.copy()
                    for qr in qr_codes:
                        (x, y, w, h) = qr.rect
                        cv2.rectangle(display_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(display_img, qr.data.decode('utf-8'), (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    cv2.imshow('QR Code Detector - Camera Feed', display_img)
                    key = cv2.waitKey(1)
                    if key == ord('q'):
                        self.display_enabled = False
                except Exception as display_err:
                    self.get_logger().debug(f'Affichage non disponible: {display_err}')
                    self.display_enabled = False
            
        except Exception as e:
            self.get_logger().error(f'Erreur traitement image: {e}')

    def odom_callback(self, msg):
        # Récupérer la position x, y et l'orientation yaw du robot
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Orientation quaternion -> yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.last_pose = (x, y, yaw)

    def lidar_callback(self, msg):
        # Prendre la mesure centrale (devant le robot)
        if len(msg.ranges) > 0:
            center_index = len(msg.ranges) // 2
            distance = msg.ranges[center_index]
            if np.isfinite(distance) and distance > 0:
                self.last_lidar_distance = distance
            else:
                self.last_lidar_distance = None


def main(args=None):
    rclpy.init(args=args)
    qr_detector = QRDetectorNode()
    try:
        rclpy.spin(qr_detector)
    except KeyboardInterrupt:
        pass
    finally:
        # Nettoyage propre
        cv2.destroyAllWindows()
        qr_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()