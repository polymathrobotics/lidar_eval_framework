# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

import json
import os
import subprocess

from lidar_eval_backends.authentication_interface import AuthInterface


# TODO: extract polymath specifc logic elsewhere

class OnePassword(AuthInterface):
    """Fetch the Google backend's credentials from 1Password via the `op` CLI and return them as
    the Google blob: the Drive `root_folder_id` plus the service-account key fields. Interactive —
    runs `op signin` if there's no live session.

    NOTE: the item title and vault are Polymath-specific. A contributor using a different secret
    source writes their own AuthInterface under database_backends/google/auth/ and enables it in
    auth_registry.yaml — no change to the storage handler.
    """

    _ITEM_TITLE = "Lidar Evaluation Results Database"
    _VAULT = "Employee"

    def authenticate(self) -> dict:
        print(f"\n--- 1Password Authentication: {self._ITEM_TITLE} ---")
        try:
            session_token = self._session_token()

            print(f"Retrieving structural mapping for: '{self._ITEM_TITLE}'...")
            item_data = json.loads(subprocess.check_output(
                ["op", "item", "get", self._ITEM_TITLE,
                 f"--session={session_token}", "--format=json"], text=True).strip())
            fields = item_data.get("fields", [])
            files = item_data.get("files", [])

            # Root Drive folder id lives on a custom field of the vault item.
            root_folder_id = self._field_value(fields, "root_folder_id")

            # Service-account key is a .json file attachment.
            file_id = next((f.get("id") for f in files
                            if f.get("name", "").lower().endswith(".json")), None)
            if not file_id:
                raise ValueError(f"No '.json' attachment found in vault item '{self._ITEM_TITLE}'.")
            secret_uri = f"op://{self._VAULT}/{self._ITEM_TITLE}/{file_id}"
            key_json = json.loads(subprocess.check_output(
                ["op", "read", secret_uri, f"--session={session_token}"]).decode("utf-8"))

            print("✅ Credentials fetched from 1Password.")
            return {"root_folder_id": root_folder_id, **key_json}

        except subprocess.CalledProcessError as e:
            print(f"\n❌ Failed to query 1Password item: {e}")
            raise
        except Exception as e:
            print(f"❌ 1Password authentication failed: {e}")
            raise

    def _session_token(self) -> str:
        """Reuse an active op session from the environment, or force a fresh (interactive) sign-in."""
        token = os.environ.get("OP_SESSION_polymathrobotics") or next(
            (val for key, val in os.environ.items() if key.startswith("OP_SESSION_")), None)
        if token:
            try:
                subprocess.check_output(["op", "account", "get", f"--session={token}"],
                                        stderr=subprocess.DEVNULL, text=True)
                print("✅ Found active 1Password terminal session token in environment.")
                return token
            except subprocess.CalledProcessError:
                print("⚠️ Stale environment token detected. Forcing clean sign-in session...")
        print("Prompting 1Password unlock...")
        token = subprocess.check_output(["op", "signin", "--raw"], text=True).strip()
        print("✅ 1Password CLI successfully unlocked.")
        return token

    @staticmethod
    def _field_value(fields: list, key: str) -> str | None:
        """Value of the 1Password custom field whose label matches `key` (ignoring spaces/underscores)."""
        norm = key.lower().replace(" ", "").replace("_", "")
        for field in fields:
            label = field.get("label", "")
            if label and norm in label.lower().replace(" ", "").replace("_", ""):
                return str(field.get("value", "")).strip()
        return None
