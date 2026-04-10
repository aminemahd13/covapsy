import json
import os
import shutil
import unittest
from unittest.mock import MagicMock, patch
import rclpy
from rclpy.node import Node
from covapsy_learning.track_learner_node import TrackLearnerNode
from covapsy_interfaces.msg import TrackQuality

class TestTrackSaving(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.test_dir = "/tmp/covapsy_test_tracks"
        if os.path.exists(self.test_dir):
            shutil.rmtree(self.test_dir)
        os.makedirs(self.test_dir)
        self.track_path = os.path.join(self.test_dir, "track.json")

    def tearDown(self):
        if os.path.exists(self.test_dir):
            shutil.rmtree(self.test_dir)

    def test_save_v2_with_rotation(self):
        """Verify that saving creates a V2 JSON and rotates old files."""
        # Setup node with mocked parameters
        node = TrackLearnerNode()
        node.track_store_path = self.track_path
        node.auto_save_on_valid = True
        node.max_backups = 3
        
        # Mock TF to avoid actual lookups
        node.tf_buffer.can_transform = MagicMock(return_value=False)

        pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]
        speeds = [0.5, 0.5, 0.5, 0.5]
        quality = TrackQuality()
        quality.is_valid = True
        quality.score = 0.9

        # Save first time
        node._save_track_to_file(pts, speeds, quality)
        self.assertTrue(os.path.exists(self.track_path))
        
        with open(self.track_path, 'r') as f:
            data = json.load(f)
            self.assertEqual(data['version'], 2)
            self.assertEqual(data['frame_id'], 'odom')
            self.assertIn('metadata', data)
            self.assertIn('track_id', data['metadata'])

        # Save again with slightly different quality to trigger save (different signature)
        quality.score = 0.91
        node._save_track_to_file(pts, speeds, quality)
        
        self.assertTrue(os.path.exists(self.track_path))
        self.assertTrue(os.path.exists(self.track_path + ".1"))

    def test_load_v1_compatibility(self):
        """Verify that a Version 1 track file can still be loaded."""
        v1_data = {
            "points": [
                {"x": 1.0, "y": 2.0, "speed_mps": 0.4},
                {"x": 2.0, "y": 3.0, "speed_mps": 0.4},
                {"x": 3.0, "y": 4.0, "speed_mps": 0.4}
            ],
            "quality": {
                "score": 0.8,
                "is_valid": True
            }
        }
        with open(self.track_path, 'w') as f:
            json.dump(v1_data, f)

        node = TrackLearnerNode()
        node.track_store_path = self.track_path
        node._load_cached_track_from_file()

        self.assertEqual(len(node.cached_track_pts), 3)
        self.assertEqual(node.cached_track_pts[0], (1.0, 2.0))
        self.assertEqual(node.cached_track_quality['score'], 0.8)

    def test_map_frame_transformation(self):
        """Verify that track is saved in 'map' frame if transform is available."""
        node = TrackLearnerNode()
        node.track_store_path = self.track_path
        node.auto_save_on_valid = True
        
        # Mock TF to simulate map frame availability
        node.tf_buffer.can_transform = MagicMock(return_value=True)
        
        # Mock _transform_path to return shifted points
        mock_transformed_pts = [(10.0, 10.0), (11.0, 10.0), (11.0, 11.0), (10.0, 10.0)]
        node._transform_path = MagicMock(return_value=mock_transformed_pts)

        pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]
        speeds = [0.5, 0.5, 0.5, 0.5]
        quality = TrackQuality()
        quality.is_valid = True
        quality.score = 0.95

        node._save_track_to_file(pts, speeds, quality)
        
        with open(self.track_path, 'r') as f:
            data = json.load(f)
            self.assertEqual(data['frame_id'], 'map')
            self.assertEqual(data['points'][0]['x'], 10.0)

if __name__ == '__main__':
    unittest.main()
