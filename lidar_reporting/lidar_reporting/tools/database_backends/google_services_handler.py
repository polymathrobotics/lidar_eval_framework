# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any

from google.oauth2 import service_account
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload


class GoogleServicesHandler:

    def __init__(self):
        self.scopes = ["https://www.googleapis.com/auth/drive"]
        self.target_item_title = "Lidar Evaluation Results Database"

        # These will be populated dynamically via 1Password authentication
        self.credentials_info = {}
        self.root_folder_id = None

        self._env_folder_ids: dict[str, str] = {}
        self._lidar_folder_ids: dict[tuple[str, str], str] = {}
        self._case_folder_ids: dict[tuple[str, str, str], str] = {}
        self._case_spreadsheet_ids: dict[tuple[str, str, str], str] = {}

    def authenticate(self):
        """Authenticates via 1Password CLI dynamically by forcing a fresh sign-in
        if the environment token is stale, downloading the raw JSON file attachment cleanly.
        """
        print(f"\n--- 1Password Authentication: {self.target_item_title} ---")

        try:
            # 1. Inspect environment space for session markers
            session_token = os.environ.get("OP_SESSION_polymathrobotics")

            if not session_token:
                session_token = next((val for key, val in os.environ.items() if key.startswith("OP_SESSION_")), None)

            # 2. Test session token validity or force active fallback sign-in
            is_token_valid = False
            if session_token:
                try:
                    # Quick validation probing call to see if token is live or expired
                    subprocess.check_output(
                        ["op", "account", "get", f"--session={session_token}"],
                        stderr=subprocess.DEVNULL,
                        text=True
                    )
                    is_token_valid = True
                    print("✅ Found active 1Password terminal session token in environment.")
                except subprocess.CalledProcessError:
                    print("⚠️ Stale environment token detected. Forcing clean sign-in session...")

            if not is_token_valid:
                print("Prompting 1Password unlock...")
                session_token = subprocess.check_output(
                    ["op", "signin", "--raw"], text=True
                ).strip()
                print("✅ 1Password CLI successfully unlocked.")

            # 3. Grab item payload data
            print(f"Retrieving structural mapping for: '{self.target_item_title}'...")
            item_json_raw = subprocess.check_output(
                [
                    "op",
                    "item",
                    "get",
                    self.target_item_title,
                    f"--session={session_token}",
                    "--format=json",
                ],
                text=True,
            ).strip()

            item_data = json.loads(item_json_raw)
            fields = item_data.get("fields", [])
            files = item_data.get("files", [])

            # 4. Pull root_folder_id from custom fields mapping
            for field in fields:
                label = field.get("label", "")
                if label and "root_folder_id" in label.lower().replace(" ", "").replace("_", ""):
                    self.root_folder_id = str(field.get("value", "")).strip()
                    break

            # Absolute fallback tracking: Route directly to your newly assigned Shared Drive
            # to prevent hitting individual storage quotas.
            if not self.root_folder_id or self.root_folder_id == "1T794rm1u6JHBMVnwEbbaAxkPtXDAM7qY":
                self.root_folder_id = "1Yod4sRoY373sEMUruFdXzlCC2fE4rzTK"

            # 5. Handle credential payload securely from Employee vault
            json_file_id = None
            for file_info in files:
                name = file_info.get("name", "").lower()
                if name.endswith(".json"):
                    json_file_id = file_info.get("id")
                    break

            if json_file_id:
                print("Found native JSON file attachment. Downloading raw key data...")

                vault_context = "Employee"
                secret_uri = f"op://{vault_context}/{self.target_item_title}/{json_file_id}"

                raw_json_bytes = subprocess.check_output(
                    [
                        "op",
                        "read",
                        secret_uri,
                        f"--session={session_token}",
                    ]
                )
                self.credentials_info = json.loads(raw_json_bytes.decode("utf-8"))
            else:
                raise ValueError("Could not find a valid credentials.json attachment in the Employee vault item.")

            print("✅ self.credentials_info successfully synchronized from file attachment.")
            print(f"✅ self.root_folder_id locked to: {self.root_folder_id}")

        except subprocess.CalledProcessError as e:
            print(f"\n❌ Failed to query 1Password item: {e}")
            raise e
        except Exception as e:
            print(f"❌ Structural parser failed to ingest data format: {e}")
            raise e

        except subprocess.CalledProcessError as e:
            print(f"\n❌ Failed to query 1Password item: {e}")
            raise e
        except Exception as e:
            print(f"❌ Structural parser failed to ingest data format: {e}")
            raise e

    def _find_child_id(self, parent_id: str, name: str, mime_type: str | None) -> str | None:
        """Returns the ID of the first non-trashed child of parent_id matching name (and mime type,
        when given), or None if no match exists. Pass mime_type=None to match a child of any type
        (e.g. an uploaded bag file). Used to make folder/sheet/file creation idempotent across runs.
        """
        creds = service_account.Credentials.from_service_account_info(
            self.credentials_info, scopes=self.scopes
        )
        service = build('drive', 'v3', credentials=creds)

        escaped_name = name.replace('\\', '\\\\').replace("'", "\\'")
        query = (
            f"'{parent_id}' in parents and "
            f"name = '{escaped_name}' and "
            f"trashed = false"
        )
        if mime_type is not None:
            query += f" and mimeType = '{mime_type}'"
        response = service.files().list(
            q=query,
            fields='files(id, name)',
            supportsAllDrives=True,
            includeItemsFromAllDrives=True,
            corpora='allDrives',
            pageSize=1,
        ).execute()
        items = response.get('files', [])
        return items[0]['id'] if items else None

    def create_environment_folder(self, env_name: str) -> str:
        """Returns the Drive folder ID for env_name, reusing an existing folder if one with that name
        already lives under root_folder_id. Falls through to creating a new folder only when no match
        is found, so repeated syncs don't pile up duplicate environment folders.
        """
        if env_name in self._env_folder_ids:
            return self._env_folder_ids[env_name]

        existing_id = self._find_child_id(
            self.root_folder_id, env_name, 'application/vnd.google-apps.folder'
        )
        if existing_id:
            print(f"♻️  Reusing existing environment folder: '{env_name}'")
            self._env_folder_ids[env_name] = existing_id
            return existing_id

        creds = service_account.Credentials.from_service_account_info(
            self.credentials_info, scopes=self.scopes
        )
        service = build('drive', 'v3', credentials=creds)

        folder_metadata = {
            'name': env_name,
            'mimeType': 'application/vnd.google-apps.folder',
            'parents': [self.root_folder_id]
        }

        print(f"Creating environment directory structural level: '{env_name}'...")
        # Added supportsAllDrives=True to allow writing to Shared Drives
        folder = service.files().create(
            body=folder_metadata,
            fields='id',
            supportsAllDrives=True
        ).execute()

        folder_id = folder.get('id')
        self._env_folder_ids[env_name] = folder_id

        return folder_id

    def create_lidar_folder(self, env_name: str, lidar_name: str) -> str:
        """Returns the Drive folder ID for env/lidar, reusing an existing lidar subfolder if one
        with that name already lives under the env folder. Same idempotent shape as
        create_environment_folder.
        """
        parent_env_id = self._env_folder_ids.get(env_name)

        if not parent_env_id:
            raise ValueError(
                f"Runtime Tracking Error: Parent environment folder for '{env_name}' does not exist in local state cache. "
                f"You must invoke create_environment_folder('{env_name}') before spinning up nested hardware targets."
            )

        if (env_name, lidar_name) in self._lidar_folder_ids:
            return self._lidar_folder_ids[(env_name, lidar_name)]

        existing_id = self._find_child_id(
            parent_env_id, lidar_name, 'application/vnd.google-apps.folder'
        )
        if existing_id:
            print(f"♻️  Reusing existing LiDAR folder: '{env_name}/{lidar_name}'")
            self._lidar_folder_ids[(env_name, lidar_name)] = existing_id
            return existing_id

        creds = service_account.Credentials.from_service_account_info(
            self.credentials_info, scopes=self.scopes
        )
        service = build('drive', 'v3', credentials=creds)

        folder_metadata = {
            'name': lidar_name,
            'mimeType': 'application/vnd.google-apps.folder',
            'parents': [parent_env_id]
        }

        print(f"Creating LiDAR nested directory: '{lidar_name}' inside environment layout '{env_name}'...")
        # Added supportsAllDrives=True here as well
        folder = service.files().create(
            body=folder_metadata,
            fields='id',
            supportsAllDrives=True
        ).execute()

        folder_id = folder.get('id')
        self._lidar_folder_ids[(env_name, lidar_name)] = folder_id

        return folder_id


    def create_case_folder(self, env_name: str, lidar_name: str, case_name: str) -> str:
        """Returns the Drive folder ID for env/lidar/case, reusing an existing case subfolder if
        one with that name already lives under the lidar folder. Same idempotent shape as
        create_lidar_folder. This is the per-case container that holds the metrics sheet and the
        case's rosbag folder side by side.
        """
        parent_lidar_id = self._lidar_folder_ids.get((env_name, lidar_name))

        if not parent_lidar_id:
            raise ValueError(
                f"Runtime Tracking Error: Nested LiDAR directory targeting '{{{env_name}: {lidar_name}}}' "
                f"not found in tracking cache. Ensure create_lidar_folder is run first."
            )

        cache_key = (env_name, lidar_name, case_name)
        if cache_key in self._case_folder_ids:
            return self._case_folder_ids[cache_key]

        existing_id = self._find_child_id(
            parent_lidar_id, case_name, 'application/vnd.google-apps.folder'
        )
        if existing_id:
            print(f"♻️  Reusing existing case folder: '{env_name}/{lidar_name}/{case_name}'")
            self._case_folder_ids[cache_key] = existing_id
            return existing_id

        creds = service_account.Credentials.from_service_account_info(
            self.credentials_info, scopes=self.scopes
        )
        service = build('drive', 'v3', credentials=creds)

        folder_metadata = {
            'name': case_name,
            'mimeType': 'application/vnd.google-apps.folder',
            'parents': [parent_lidar_id],
        }

        print(f"Creating case directory: '{case_name}' inside '{env_name}/{lidar_name}/'...")
        folder = service.files().create(
            body=folder_metadata,
            fields='id',
            supportsAllDrives=True,
        ).execute()

        folder_id = folder.get('id')
        self._case_folder_ids[cache_key] = folder_id
        return folder_id

    def create_folder_per_case(
        self,
        env_name: str,
        lidar_name: str,
        case_name: str,
        metrics: dict[str, Any],
        bag_dirs: list | None = None,
    ) -> str:
        """Creates (or reuses) a per-case Drive folder and drops two things inside it: the metrics
        Google Sheet and the case's rosbag folder(s). Returns the case folder's Drive URL.

        Idempotent: an existing sheet is reused (metrics write skipped) and bag files already on
        Drive are not re-uploaded, so repeated syncs don't duplicate work or overwrite prior runs.
        """
        case_folder_id = self.create_case_folder(env_name, lidar_name, case_name)

        creds = service_account.Credentials.from_service_account_info(
            self.credentials_info, scopes=self.scopes
        )
        service = build('drive', 'v3', credentials=creds)

        self._write_case_sheet(service, creds, case_folder_id, env_name, lidar_name, case_name, metrics)

        for bag_dir in (bag_dirs or []):
            self._upload_bag_folder(service, case_folder_id, Path(bag_dir))

        folder = service.files().get(
            fileId=case_folder_id, fields='webViewLink', supportsAllDrives=True
        ).execute()
        return folder.get('webViewLink', '')

    def _write_case_sheet(
        self, service, creds, case_folder_id: str, env_name: str, lidar_name: str,
        case_name: str, metrics: dict[str, Any],
    ) -> str:
        """Creates the metrics spreadsheet inside the case folder and writes the metrics row.
        Reuses an existing sheet (skipping the write) when one is already present.
        """
        sheet_name = f"{lidar_name} - {case_name}"
        cache_key = (env_name, lidar_name, case_name)

        if cache_key in self._case_spreadsheet_ids:
            return self._case_spreadsheet_ids[cache_key]

        existing_id = self._find_child_id(
            case_folder_id, sheet_name, 'application/vnd.google-apps.spreadsheet'
        )
        if existing_id:
            print(f"♻️  Reusing existing case sheet: '{sheet_name}' — skipping metrics write")
            self._case_spreadsheet_ids[cache_key] = existing_id
            return existing_id

        sheet_metadata = {
            'name': sheet_name,
            'mimeType': 'application/vnd.google-apps.spreadsheet',
            'parents': [case_folder_id],
        }

        print(f"Creating evaluation Google Sheet for case '{case_name}' inside '{env_name}/{lidar_name}/{case_name}/'...")
        spreadsheet_file = service.files().create(
            body=sheet_metadata,
            fields='id, webViewLink',
            supportsAllDrives=True,
        ).execute()
        spreadsheet_id = spreadsheet_file.get('id')
        self._case_spreadsheet_ids[cache_key] = spreadsheet_id

        sheets_service = build('sheets', 'v4', credentials=creds)

        headers = ["Case Name"] + list(metrics.keys())
        values = [case_name] + list(metrics.values())
        sheets_service.spreadsheets().values().append(
            spreadsheetId=spreadsheet_id,
            range="Sheet1!A1",
            valueInputOption="USER_ENTERED",
            insertDataOption="INSERT_ROWS",
            body={'values': [headers, values]},
        ).execute()

        print(f"🚀 Metrics successfully recorded to Sheet for case '{case_name}'.")
        return spreadsheet_id

    def write_environment_summary(
        self, env_name: str, cases: list[tuple[str, str, dict[str, Any]]]
    ) -> str | None:
        """Writes/overwrites a single "<env> - summary" spreadsheet in the environment folder
        with one row per case (columns: Lidar, Case Name, then the flattened metric keys).

        This lets the dashboard load an entire environment's metrics in ONE read instead of
        one read per case spreadsheet. The per-case sheets and their visualization tabs are
        unchanged — this summary is purely additive.

        cases: list of (lidar_name, case_name, flat_metrics), where flat_metrics is the same
        slash-joined `{zone}/{Metric}/{sub}: value` mapping written to the per-case sheet.
        """
        env_folder_id = self._env_folder_ids.get(env_name)
        if not env_folder_id:
            raise ValueError(
                f"Runtime Tracking Error: environment folder for '{env_name}' not in cache; "
                f"call create_environment_folder('{env_name}') first."
            )
        if not cases:
            return None

        # Union of metric columns, preserving first-seen order across cases.
        columns: list[str] = []
        seen: set[str] = set()
        for _, _, flat_metrics in cases:
            for key in flat_metrics:
                if key not in seen:
                    seen.add(key)
                    columns.append(key)
        rows = [['Lidar', 'Case Name'] + columns]
        for lidar_name, case_name, flat_metrics in cases:
            rows.append([lidar_name, case_name] + [flat_metrics.get(c, '') for c in columns])

        creds = service_account.Credentials.from_service_account_info(
            self.credentials_info, scopes=self.scopes
        )
        drive_service = build('drive', 'v3', credentials=creds)
        sheets_service = build('sheets', 'v4', credentials=creds)

        summary_name = f"{env_name} - summary"
        spreadsheet_id = self._find_child_id(
            env_folder_id, summary_name, 'application/vnd.google-apps.spreadsheet'
        )
        if spreadsheet_id:
            # Clear stale content so a re-sync with fewer cases/columns leaves no orphans.
            sheets_service.spreadsheets().values().clear(
                spreadsheetId=spreadsheet_id, range='Sheet1', body={},
            ).execute()
        else:
            created = drive_service.files().create(
                body={
                    'name': summary_name,
                    'mimeType': 'application/vnd.google-apps.spreadsheet',
                    'parents': [env_folder_id],
                },
                fields='id',
                supportsAllDrives=True,
            ).execute()
            spreadsheet_id = created.get('id')

        sheets_service.spreadsheets().values().update(
            spreadsheetId=spreadsheet_id,
            range='Sheet1!A1',
            valueInputOption='USER_ENTERED',
            body={'values': rows},
        ).execute()
        print(f"🚀 Environment summary written: '{summary_name}' ({len(cases)} cases).")
        return spreadsheet_id

    def _upload_bag_folder(self, service, case_folder_id: str, bag_dir: Path) -> None:
        """Zips a rosbag folder and uploads it as a single '<bagname>.zip' into the case folder,
        then shares it 'anyone with the link: reader' so the dashboard can offer a direct download
        URL. Reuses an existing zip if present (just re-asserts the share) so re-syncs don't
        re-zip/re-upload gigabytes.
        """
        if not bag_dir.is_dir():
            print(f"⚠️  Bag folder missing, skipping upload: {bag_dir}")
            return

        zip_name = f"{bag_dir.name}.zip"
        existing_id = self._find_child_id(case_folder_id, zip_name, None)
        if existing_id:
            print(f"♻️  Bag zip already on Drive, skipping upload: '{zip_name}'")
            self._share_anyone(service, existing_id)
            return

        with tempfile.TemporaryDirectory() as tmp:
            print(f"🗜️  Zipping bag folder '{bag_dir.name}'...")
            # base_dir=bag_dir.name keeps the rosbag2_* folder *inside* the archive,
            # so unzipping yields a ready-to-play bag directory rather than loose files.
            zip_path = shutil.make_archive(
                os.path.join(tmp, bag_dir.name), 'zip',
                root_dir=str(bag_dir.parent), base_dir=bag_dir.name,
            )

            # resumable=True so a large zip uploads in chunks rather than one shot.
            media = MediaFileUpload(zip_path, mimetype='application/zip', resumable=True)
            print(f"⬆️  Uploading '{zip_name}' ({os.path.getsize(zip_path) / 1e6:.1f} MB)...")
            uploaded = service.files().create(
                body={'name': zip_name, 'parents': [case_folder_id]},
                media_body=media,
                fields='id',
                supportsAllDrives=True,
            ).execute()

        self._share_anyone(service, uploaded.get('id'))
        print(f"🚀 Bag zip '{zip_name}' synced to Drive and shared via link.")

    def _share_anyone(self, service, file_id: str) -> None:
        """Grants 'anyone with the link: reader' on a file so it has a public download URL.
        Idempotent — Drive no-ops (or errors harmlessly) if the permission already exists.
        """
        try:
            service.permissions().create(
                fileId=file_id,
                body={'type': 'anyone', 'role': 'reader'},
                supportsAllDrives=True,
            ).execute()
        except Exception as e:
            print(f"⚠️  Could not set public-link permission on {file_id}: {e}")

    def push_visualization_to_case(
        self, env_name: str, lidar_name: str, case_name: str, blocks: list[list[str]]
    ) -> None:
        """Adds (or clears + reuses) a 'Visualization' tab inside the case's existing spreadsheet
        and writes the supplied rows into it. Mirrors the Notion sub-page approach: viz data lives
        alongside the metrics for the same case rather than in a separate file.
        """
        rows = self._blocks_to_rows(blocks)
        if not rows:
            return

        spreadsheet_id = self._case_spreadsheet_ids.get((env_name, lidar_name, case_name))

        if not spreadsheet_id:
            raise ValueError(
                f"Runtime Tracking Error: No spreadsheet found for case "
                f"'{env_name}/{lidar_name}/{case_name}' in tracking cache. "
                f"Ensure create_folder_per_case is run first."
            )

        creds = service_account.Credentials.from_service_account_info(
            self.credentials_info, scopes=self.scopes
        )
        sheets_service = build('sheets', 'v4', credentials=creds)

        viz_title = 'Visualization'
        spreadsheet = sheets_service.spreadsheets().get(spreadsheetId=spreadsheet_id).execute()
        existing_titles = {s['properties']['title'] for s in spreadsheet.get('sheets', [])}

        if viz_title in existing_titles:
            sheets_service.spreadsheets().values().clear(
                spreadsheetId=spreadsheet_id,
                range=f'{viz_title}!A:Z',
                body={},
            ).execute()
        else:
            sheets_service.spreadsheets().batchUpdate(
                spreadsheetId=spreadsheet_id,
                body={'requests': [{'addSheet': {'properties': {'title': viz_title}}}]},
            ).execute()

        if not rows:
            return

        sheets_service.spreadsheets().values().append(
            spreadsheetId=spreadsheet_id,
            range=f'{viz_title}!A1',
            # RAW (not USER_ENTERED): viz rows are plain text — headings, "key: value"
            # bullets, and base64 cloud chunks. base64 uses '+', so a chunk starting
            # with '+' (or '=' / '-' / '@') would be parsed as a formula under
            # USER_ENTERED and corrupt the cloud. Large clouds (e.g. Seyond Robin)
            # have hundreds of chunks, so this is near-certain to mangle one.
            valueInputOption='RAW',
            insertDataOption='INSERT_ROWS',
            body={'values': rows},
        ).execute()

        print(f"🚀 Visualization pushed to '{viz_title}' tab of case '{case_name}'.")



    def sync(
        self,
        test_data: dict[str, dict[str, dict[str, Any]]],
        rosbags: dict[str, dict[str, list]] | None = None,
    ) -> None:

        rosbags = rosbags or {}
        for env_name, lidar_data in test_data.items():
            self.create_environment_folder(env_name)
            summary_cases: list[tuple[str, str, dict[str, Any]]] = []
            for lidar_name, case_data in lidar_data.items():
                self.create_lidar_folder(env_name, lidar_name)
                for case_path, metrics in case_data.items():
                    flat_metrics = self._flatten_metrics(metrics)
                    if not flat_metrics:
                        continue
                    merged = {**self._lidar_metadata, **flat_metrics}
                    # Bags are keyed by (lidar, case) only — the bag root is a single
                    # env, so there's no env level to match. Missing → no bag uploaded.
                    case_bags = rosbags.get(lidar_name, {}).get(case_path, [])
                    self.create_folder_per_case(
                        env_name, lidar_name, case_path, merged, case_bags
                    )
                    summary_cases.append((lidar_name, case_path, merged))
            # One summary sheet per env (one row per case) so the dashboard loads the whole
            # environment in a single read instead of one read per case.
            self.write_environment_summary(env_name, summary_cases)


    @staticmethod
    def _blocks_to_rows(blocks: list) -> list[list[str]]:
        rows: list[list[str]] = []
        for block in blocks:
            btype = block.get('type')
            if not btype:
                continue
            rich_text = block.get(btype, {}).get('rich_text', [])
            if not rich_text:
                continue
            text = rich_text[0].get('text', {}).get('content', '')
            if not text:
                continue
            rows.append([text])
        return rows
