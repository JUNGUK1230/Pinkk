from datetime import datetime, timedelta, timezone
import importlib
import os
import sys
import tempfile
import unittest


sys.path.insert(0, "src")
os.environ["PINKK_ENABLE_VISION"] = "0"
os.environ["PINKK_API_KEY"] = "test-secret"
os.environ["PINKK_ADMIN_KEY"] = "admin-secret"
_command_db_dir = tempfile.TemporaryDirectory()
os.environ["PINKK_COMMAND_DB_PATH"] = os.path.join(_command_db_dir.name, "commands.db")

from central_control.parking_management_web.robot_status_store import RobotStatusStore


class RobotStatusStoreTest(unittest.TestCase):
    def test_connection_thresholds(self):
        store = RobotStatusStore()
        now = datetime(2026, 7, 27, tzinfo=timezone.utc)
        payload = {
            "robot_id": "pinky1",
            "battery_percent": 82.0,
            "status": "idle",
            "charging": False,
        }
        store.update(payload, now)
        self.assertEqual(store.snapshot(now + timedelta(seconds=4.9))["pinky1"]["connection"], "online")
        self.assertEqual(store.snapshot(now + timedelta(seconds=5))["pinky1"]["connection"], "delayed")
        self.assertEqual(store.snapshot(now + timedelta(seconds=10))["pinky1"]["connection"], "offline")


class RobotStatusApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        module = importlib.import_module("central_control.parking_management_web.app_management_battery")
        module.app.config.update(TESTING=True)
        cls.client = module.app.test_client()

    def test_rejects_missing_api_key(self):
        response = self.client.post("/api/robots/status", json=self.payload())
        self.assertEqual(response.status_code, 401)

    def test_validates_and_returns_both_robots(self):
        response = self.client.post(
            "/api/robots/status",
            json=self.payload(),
            headers={"X-API-Key": "test-secret"},
        )
        self.assertEqual(response.status_code, 201)
        result = self.client.get("/api/robots/status")
        self.assertEqual(result.status_code, 200)
        data = result.get_json()
        self.assertEqual(data["pinky1"]["battery_percent"], 82.0)
        self.assertIn("pinky2", data)
        self.assertFalse(data["pinky2"]["online"])

    def test_rejects_out_of_range_battery(self):
        payload = self.payload()
        payload["battery_percent"] = 101
        response = self.client.post(
            "/api/robots/status",
            json=payload,
            headers={"X-API-Key": "test-secret"},
        )
        self.assertEqual(response.status_code, 400)

    def test_command_is_saved_claimed_and_completed(self):
        created = self.client.post(
            "/api/robots/pinky2/commands",
            json={"command": "entry", "parameters": {"parking_slot": "P8"}},
            headers={"X-Admin-Key": "admin-secret"},
        )
        self.assertEqual(created.status_code, 201)
        command_id = created.get_json()["command_id"]

        claimed = self.client.get(
            "/api/robots/pinky2/commands/next",
            headers={"X-API-Key": "test-secret"},
        )
        self.assertEqual(claimed.status_code, 200)
        self.assertEqual(claimed.get_json()["command_id"], command_id)
        self.assertEqual(claimed.get_json()["state"], "delivered")

        completed = self.client.post(
            f"/api/robots/pinky2/commands/{command_id}/result",
            json={"state": "completed", "message": "ROS 명령 토픽에 전달됨"},
            headers={"X-API-Key": "test-secret"},
        )
        self.assertEqual(completed.status_code, 200)
        self.assertEqual(completed.get_json()["state"], "completed")

        history = self.client.get(
            "/api/robots/commands/history",
            headers={"X-Admin-Key": "admin-secret"},
        )
        self.assertEqual(history.status_code, 200)
        self.assertTrue(any(item["command_id"] == command_id for item in history.get_json()["commands"]))

    def test_command_requires_admin_key_and_known_command(self):
        unauthorized = self.client.post(
            "/api/robots/pinky1/commands", json={"command": "emergency_stop"}
        )
        self.assertEqual(unauthorized.status_code, 401)
        invalid = self.client.post(
            "/api/robots/pinky1/commands",
            json={"command": "fly"},
            headers={"X-Admin-Key": "admin-secret"},
        )
        self.assertEqual(invalid.status_code, 400)

    @staticmethod
    def payload():
        return {
            "robot_id": "pinky1",
            "battery_percent": 82.0,
            "battery_voltage": 12.3,
            "status": "idle",
            "charging": False,
            "timestamp": "2026-07-27T10:00:00+09:00",
        }


if __name__ == "__main__":
    unittest.main()
