import json
import os
import shutil
import unittest
from unittest.mock import MagicMock
import hashlib
from datetime import datetime

# --- MOCKING ROS 2 PARTS ---
class MockHeader:
    def __init__(self):
        self.stamp = None
        self.frame_id = ""

class MockTrackQuality:
    def __init__(self):
        self.header = MockHeader()
        self.score = 0.0
        self.closure_error_m = 0.0
        self.smoothness = 0.0
        self.spacing_stddev = 0.0
        self.lap_consistency = 0.0
        self.is_valid = False
        self.reason = ""

# Mocking the TrackLearnerNode to test its methods in isolation
# We'll just import the logic from the actual file but mock the base class
import sys
from types import ModuleType

# Mock the ROS modules before they are imported by track_learner_node
mock_rclpy = MagicMock()
mock_rclpy_duration = MagicMock()
mock_rclpy_node = MagicMock()
mock_tf2_ros = MagicMock()
mock_tf2_geom = MagicMock()
mock_interfaces = MagicMock()
mock_geom_msgs = MagicMock()
mock_nav_msgs = MagicMock()
mock_std_msgs = MagicMock()
mock_sensor_msgs = MagicMock()

# Essential: Mock Node to be a class that TrackLearnerNode can inherit from
class MockNode:
    def __init__(self, *args, **kwargs):
        pass
    def declare_parameter(self, *args, **kwargs):
        pass
    def get_parameter(self, name):
        m = MagicMock()
        m.value = ""
        return m
    def create_subscription(self, *args, **kwargs):
        return MagicMock()
    def create_publisher(self, *args, **kwargs):
        return MagicMock()
    def create_timer(self, *args, **kwargs):
        return MagicMock()
    def get_logger(self):
        return MagicMock()
    def get_clock(self):
        m = MagicMock()
        m.now().to_msg.return_value = None
        return m

mock_rclpy_node.Node = MockNode

sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.duration'] = mock_rclpy_duration
sys.modules['rclpy.node'] = mock_rclpy_node
sys.modules['tf2_ros'] = mock_tf2_ros
sys.modules['tf2_geometry_msgs'] = mock_tf2_geom
sys.modules['tf2_geometry_msgs.tf2_geometry_msgs'] = MagicMock()
sys.modules['covapsy_interfaces.msg'] = mock_interfaces
sys.modules['geometry_msgs.msg'] = mock_geom_msgs
sys.modules['nav_msgs.msg'] = mock_nav_msgs
sys.modules['std_msgs.msg'] = mock_std_msgs
sys.modules['sensor_msgs.msg'] = mock_sensor_msgs
mock_interfaces.TrackQuality = MockTrackQuality
mock_interfaces.DirectionState = MagicMock()

# Now we can import the node logic (it will use our mocks)
# We need to add the source directory to the path
sys.path.append(os.path.join(os.getcwd(), 'ros2_ws/src/covapsy_learning'))
from covapsy_learning.track_learner_node import TrackLearnerNode

# --- THE TEST CLASS ---

class TestTrackSavingLogic(unittest.TestCase):
    def setUp(self):
        self.test_dir = os.path.join(os.getcwd(), "test_tmp_covapsy")
        if os.path.exists(self.test_dir):
            shutil.rmtree(self.test_dir)
        os.makedirs(self.test_dir)
        self.track_path = os.path.join(self.test_dir, "track.json")

        # Create a node instance without calling __init__
        self.node = TrackLearnerNode.__new__(TrackLearnerNode)
        
        # Manually set attributes used in tested methods
        self.node.track_store_path = self.track_path
        self.node.auto_save_on_valid = True
        self.node.max_backups = 3
        self.node.sample_distance = 0.05
        self.node.speed_max = 0.6
        self.node.get_logger = MagicMock()
        self.node.tf_buffer = MagicMock()
        self.node._last_saved_signature = ''
        self.node.cached_track_pts = []
        self.node.cached_track_speeds = []
        self.node.cached_track_quality = None
        self.node.freeze_after_valid = False
        self.node.track_frozen = False
        self.node.get_clock = MagicMock()
        self.node.get_clock().now().to_msg.return_value = None
        
        # Bind the methods we want to test to the instance
        # Since we used __new__, we need to make sure internal helper methods work.
        # TrackLearnerNode._quality_to_dict is a regular method, so it's fine.
        self.node._quality_to_dict = TrackLearnerNode._quality_to_dict.__get__(self.node, TrackLearnerNode)
        self.node._quality_from_dict = TrackLearnerNode._quality_from_dict.__get__(self.node, TrackLearnerNode)
        self.node._save_track_to_file = TrackLearnerNode._save_track_to_file.__get__(self.node, TrackLearnerNode)
        self.node._load_cached_track_from_file = TrackLearnerNode._load_cached_track_from_file.__get__(self.node, TrackLearnerNode)
        self.node._cache_valid_track = TrackLearnerNode._cache_valid_track.__get__(self.node, TrackLearnerNode)

    def tearDown(self):
        # We'll leave the test_tmp_covapsy/ folder for manual inspection if needed.
        # Use shutil.rmtree(self.test_dir) to clean up manually.
        pass

    def test_save_v2_logic(self):
        """Test the save logic, file rotation, and JSON schema versioning."""
        pts = [(0.0, 0.0), (1.0, 1.0)]
        speeds = [0.5, 0.5]
        quality = MockTrackQuality()
        quality.is_valid = True
        quality.score = 0.9
        
        # Mock TF to say 'no map frame'
        self.node.tf_buffer.can_transform.return_value = False

        # 1. First Save
        self.node._save_track_to_file(pts, speeds, quality)
        self.assertTrue(os.path.exists(self.track_path))
        
        with open(self.track_path, 'r') as f:
            data = json.load(f)
            self.assertEqual(data['version'], 2)
            self.assertEqual(data['frame_id'], 'odom')
            self.assertIn('track_id', data['metadata'])
            self.assertEqual(len(data['points']), 2)

        # 2. Second Save (Rotation)
        # Change signature slightly to trigger save
        quality.score = 0.91
        self.node._save_track_to_file(pts, speeds, quality)
        
        self.assertTrue(os.path.exists(self.track_path))
        self.assertTrue(os.path.exists(self.track_path + ".1"))
        
        # 3. Multiple Rotations
        for i in range(2, 5):
            quality.score = 0.9 + i*0.01
            self.node._save_track_to_file(pts, speeds, quality)
            
        self.assertTrue(os.path.exists(self.track_path + ".1"))
        self.assertTrue(os.path.exists(self.track_path + ".2"))
        self.assertTrue(os.path.exists(self.track_path + ".3"))
        # Max backups is 3, so .4 shouldn't exist
        self.assertFalse(os.path.exists(self.track_path + ".4"))

    def test_load_compatibility(self):
        """Test loading both V1 and V2 formats."""
        # Save a V1-like file
        v1_data = {
            "points": [
                {"x": 1.0, "y": 1.0, "speed_mps": 0.5},
                {"x": 2.0, "y": 2.0, "speed_mps": 0.5},
                {"x": 3.0, "y": 3.0, "speed_mps": 0.5}
            ],
            "quality": {"score": 0.8, "is_valid": True}
        }
        with open(self.track_path, 'w') as f:
            json.dump(v1_data, f)
            
        self.node._load_cached_track_from_file()
        self.assertEqual(len(self.node.cached_track_pts), 3)
        self.assertEqual(self.node._loaded_frame_id, 'odom') # Default for V1

        # Save a V2-like file in map frame
        v2_data = {
            "version": 2,
            "frame_id": "map",
            "metadata": {"track_id": "test"},
            "points": [
                {"x": 10.0, "y": 10.0, "speed_mps": 0.5},
                {"x": 11.0, "y": 11.0, "speed_mps": 0.5},
                {"x": 12.0, "y": 12.0, "speed_mps": 0.5}
            ],
            "quality": {"score": 0.95, "is_valid": True}
        }
        with open(self.track_path, 'w') as f:
            json.dump(v2_data, f)
            
        self.node._load_cached_track_from_file()
        self.assertEqual(self.node.cached_track_pts[0], (10.0, 10.0))
        self.assertEqual(self.node._loaded_frame_id, 'map')

    def test_map_frame_decision(self):
        """Test if the node chooses the 'map' frame when TF is available."""
        pts = [(0.0, 0.0)]
        speeds = [0.5]
        quality = MockTrackQuality()
        quality.is_valid = True
        
        # Mock TF to say 'map is available'
        self.node.tf_buffer.can_transform.return_value = True
        # Mock the transform method to return shifted points
        self.node._transform_path = MagicMock(return_value=[(100.0, 100.0)])
        
        self.node._save_track_to_file(pts, speeds, quality)
        
        with open(self.track_path, 'r') as f:
            data = json.load(f)
            self.assertEqual(data['frame_id'], 'map')
            self.assertEqual(data['points'][0]['x'], 100.0)

from unittest.mock import patch
if __name__ == '__main__':
    unittest.main()
