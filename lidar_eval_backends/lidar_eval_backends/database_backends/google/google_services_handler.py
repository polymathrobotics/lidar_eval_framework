# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

import base64
import importlib
import os
import tempfile
import threading
import time
import zipfile
from concurrent.futures import ThreadPoolExecutor
from importlib import resources
from pathlib import Path
from typing import Any

import httplib2
import numpy as np
import yaml
from google.oauth2 import service_account
from google_auth_httplib2 import AuthorizedHttp
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError
from googleapiclient.http import MediaFileUpload

from lidar_eval_backends.database_interface import DatabaseInterface


class GoogleServicesHandler(DatabaseInterface):

    # Read-path tuning + Drive/Sheets mime constants (used by the retrieve_* methods).
    _NUM_RETRIES = 4
    _MAX_WORKERS = 5
    _READ_ATTEMPTS = 3
    _PARENTS_PER_QUERY = 50
    _FOLDER_MIME = 'application/vnd.google-apps.folder'
    _SHEET_MIME = 'application/vnd.google-apps.spreadsheet'
    _SCOPES = [
        'https://www.googleapis.com/auth/drive',
        'https://www.googleapis.com/auth/spreadsheets',
    ]

    def __init__(self):
        self.scopes = ["https://www.googleapis.com/auth/drive"]

        # Populated by authenticate() (via a credential provider, write path) or
        # load_credentials() (PolyView's injected blob, read path).
        self.credentials_info = {}
        self.root_folder_id = None
        self._cached_read_creds = None

        self._env_folder_ids: dict[str, str] = {}
        self._lidar_folder_ids: dict[tuple[str, str], str] = {}
        self._case_folder_ids: dict[tuple[str, str, str], str] = {}
        self._case_spreadsheet_ids: dict[tuple[str, str, str], str] = {}

        # Read-path state: thread-local Drive/Sheets clients (httplib2 isn't thread-safe)
        # plus a (parent, mime) -> {name: id} listing cache shared across reads.
        self._thread_local = threading.local()
        self._children_cache: dict[tuple[str, str], dict[str, str]] = {}
        self._cache_lock = threading.Lock()

    def authenticate(self) -> None:
        """Obtain this backend's credentials via its configured credential provider (see auth/ +
        auth_registry.yaml) and load them. The read path (PolyView) skips this and injects
        credentials directly via load_credentials()."""
        provider = self._resolve_auth_provider()
        self.load_credentials(provider.authenticate())

    def _resolve_auth_provider(self):
        # Resolve the enabled credential provider from this backend's own auth_registry.yaml.
        # Auth is a Google-internal concern here: swap it by adding a provider under auth/ and
        # flipping the registry — the storage logic below never changes.
        registry_path = resources.files(
            'lidar_eval_backends.database_backends.google') / 'auth_registry.yaml'
        with registry_path.open('r') as f:
            config = yaml.safe_load(f)
        for provider in config.get('authentication_registry', []):
            if provider.get('enabled') is True:
                module = importlib.import_module(
                    f"lidar_eval_backends.database_backends.google.auth.{provider['executable']}")
                # The row's `config` block travels with the provider, so a provider never has to
                # work out where its own credentials live.
                return getattr(module, provider['class'])(provider.get('config'))
        raise RuntimeError("No enabled authentication provider in google/auth_registry.yaml")

    def load_credentials(self, credentials: dict) -> None:
        """Split an opaque credential blob (injected by PolyView, or built on the write path)
        into what this backend needs: the Drive root folder id and the service-account fields."""
        blob = dict(credentials)
        self.root_folder_id = blob.pop('root_folder_id', None)
        self.credentials_info = blob
        self._cached_read_creds = None   # invalidate any memoized creds

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

        # A bag upload can fail (network, oversized file) independently of the metrics/viz that
        # already succeeded above — isolate it so one bad bag doesn't abort the rest of sync().
        for bag_dir in (bag_dirs or []):
            try:
                self._upload_bag_folder(service, case_folder_id, Path(bag_dir))
            except Exception as e:
                # flush + full traceback: this runs under ros2 launch where stdout is block-
                # buffered, so an un-flushed message never reaches the log and the failure looks
                # like a silent success ("sync complete" with no bag on Drive).
                import traceback
                print(f"❌ Bag upload FAILED for '{Path(bag_dir).name}': {type(e).__name__}: {e}",
                      flush=True)
                print(traceback.format_exc(), flush=True)

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
        """Merges the given cases into the single "<env> - summary" spreadsheet in the
        environment folder — one row per (lidar, case), columns: Lidar, Case Name, then the
        flattened metric keys.

        This lets the dashboard load an entire environment's metrics in ONE read instead of
        one read per case spreadsheet. The per-case sheets and their visualization tabs are
        unchanged — this summary is purely additive.

        MERGE, not overwrite: a bench run syncs a single lidar, so this is called with only
        that lidar's cases. Overwriting would drop every previously-synced lidar from the
        summary (and the dashboard's summary fast-path would then hide them even though their
        per-case sheets still exist on Drive). We read the existing summary, key its rows by
        (lidar, case), overlay the current run's cases (updating same keys, inserting new
        ones), and rewrite the union — so other lidars' cases are preserved.

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

        creds = service_account.Credentials.from_service_account_info(
            self.credentials_info, scopes=self.scopes
        )
        drive_service = build('drive', 'v3', credentials=creds)
        sheets_service = build('sheets', 'v4', credentials=creds)

        summary_name = f"{env_name} - summary"
        spreadsheet_id = self._find_child_id(
            env_folder_id, summary_name, 'application/vnd.google-apps.spreadsheet'
        )

        # Column order and row map, seeded from the existing summary so prior lidars survive.
        columns: list[str] = []
        seen: set[str] = set()
        merged: dict[tuple[str, str], dict[str, Any]] = {}

        def _track_columns(keys) -> None:
            for key in keys:
                if key not in seen:
                    seen.add(key)
                    columns.append(key)

        if spreadsheet_id:
            existing_rows = self._read_summary_rows(sheets_service, spreadsheet_id)
            if len(existing_rows) >= 2:
                header = [str(h) for h in existing_rows[0][2:]]
                _track_columns(header)
                for row in existing_rows[1:]:
                    if len(row) < 2:
                        continue
                    key = (str(row[0]), str(row[1]))
                    values = row[2:]
                    # Trailing empty cells are dropped by Sheets, so a row can be shorter than
                    # the header; skip '' so a lidar's blank columns don't shadow another's.
                    merged[key] = {
                        col: values[i]
                        for i, col in enumerate(header)
                        if i < len(values) and values[i] != ''
                    }

        # Overlay this run's cases: same (lidar, case) is replaced, new ones are inserted.
        for lidar_name, case_name, flat_metrics in cases:
            _track_columns(flat_metrics.keys())
            merged[(lidar_name, case_name)] = dict(flat_metrics)

        rows = [['Lidar', 'Case Name'] + columns]
        for (lidar_name, case_name), flat_metrics in merged.items():
            rows.append([lidar_name, case_name] + [flat_metrics.get(c, '') for c in columns])

        if spreadsheet_id:
            # Clear stale content so the rewrite leaves no orphan rows/columns from a prior
            # (possibly wider) summary; the merged `rows` above already carry every prior case.
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
        print(f"🚀 Environment summary written: '{summary_name}' "
              f"({len(cases)} case(s) this run, {len(merged)} total).")
        return spreadsheet_id

    @staticmethod
    def _read_summary_rows(sheets_service, spreadsheet_id: str) -> list[list]:
        """Returns the existing summary's rows (header + data), or [] if it can't be read.
        A read failure is treated as 'no prior rows' — safer to fall back to writing just this
        run than to abort the whole sync, and the dashboard's per-case fallback still surfaces
        any lidar the summary ends up missing."""
        try:
            response = sheets_service.spreadsheets().values().get(
                spreadsheetId=spreadsheet_id,
                range='Sheet1',
                valueRenderOption='UNFORMATTED_VALUE',
            ).execute()
        except (HttpError, OSError):
            return []
        return response.get('values', [])

    def _upload_bag_folder(self, service, case_folder_id: str, bag_dir: Path) -> None:
        """Zips a rosbag folder and uploads it as a single '<bagname>.zip' into the case folder,
        then shares it 'anyone with the link: reader' so the dashboard can offer a direct download
        URL. Reuses an existing zip if present (just re-asserts the share) so re-syncs don't
        re-zip/re-upload gigabytes.

        Bags are multi-GB and their .mcap payload is already compressed, so the zip is written
        STORED (no DEFLATE) — bundling only, no wasted CPU — and uploaded in chunks with retry.
        A single-shot resumable .execute() on a >1 GB file reliably trips httplib2's
        "Redirected but the response is missing a Location: header"; chunked next_chunk() with
        num_retries rides out the transient redirects/5xx that a long upload hits.
        """
        if not bag_dir.is_dir():
            print(f"⚠️  Bag folder missing, skipping upload: {bag_dir}", flush=True)
            return

        # Expected archive size ≈ sum of the bag's files (STORED = no compression, so this is
        # within a few KB of the real zip). Used to detect a partial/zero-byte leftover from a
        # previously-failed upload so we re-upload it instead of skipping a broken file.
        local_size = sum(f.stat().st_size for f in bag_dir.rglob('*') if f.is_file())

        zip_name = f"{bag_dir.name}.zip"
        print(f"📦 Bag upload starting: '{zip_name}' → case folder {case_folder_id} "
              f"(source {local_size / 1e6:.1f} MB)", flush=True)
        existing_id = self._find_child_id(case_folder_id, zip_name, None)
        if existing_id:
            remote_size = self._drive_file_size(service, existing_id)
            # Treat within 1% of expected as a complete prior upload; anything smaller is a
            # partial/corrupt leftover — delete it and re-upload rather than skip.
            if remote_size is not None and remote_size >= local_size * 0.99:
                print(f"♻️  Bag zip already complete on Drive ({remote_size / 1e6:.1f} MB), "
                      f"skipping upload: '{zip_name}'", flush=True)
                self._share_anyone(service, existing_id)
                return
            print(f"⚠️  Found INCOMPLETE bag zip on Drive "
                  f"({(remote_size or 0) / 1e6:.1f} MB vs expected {local_size / 1e6:.1f} MB) — "
                  f"deleting and re-uploading: '{zip_name}'", flush=True)
            try:
                service.files().delete(fileId=existing_id, supportsAllDrives=True).execute()
            except Exception as e:
                print(f"⚠️  Could not delete stale zip {existing_id}: {e}", flush=True)

        with tempfile.TemporaryDirectory() as tmp:
            zip_path = os.path.join(tmp, zip_name)
            print(f"🗜️  Bundling bag folder '{bag_dir.name}' (stored, no compression)...", flush=True)
            # arcname is relative to bag_dir.parent so the rosbag2_* folder stays the archive's
            # top-level entry — unzipping yields a ready-to-play bag directory, not loose files.
            with zipfile.ZipFile(zip_path, 'w', compression=zipfile.ZIP_STORED, allowZip64=True) as zf:
                for file_path in sorted(bag_dir.rglob('*')):
                    if file_path.is_file():
                        zf.write(file_path, file_path.relative_to(bag_dir.parent))

            # chunksize 100 MB: large enough to keep overhead low, small enough that a failed
            # chunk retries cheaply. resumable=True is required for next_chunk().
            media = MediaFileUpload(
                zip_path, mimetype='application/zip', resumable=True, chunksize=100 * 1024 * 1024
            )
            print(f"⬆️  Uploading '{zip_name}' ({os.path.getsize(zip_path) / 1e6:.1f} MB)...", flush=True)
            request = service.files().create(
                body={'name': zip_name, 'parents': [case_folder_id]},
                media_body=media,
                fields='id',
                supportsAllDrives=True,
            )
            # Google returns "308 Resume Incomplete" between resumable chunks — it carries a
            # Range header but NO Location. httplib2 treats 308 as a redirect to follow and,
            # finding no Location, raises RedirectMissingLocation, aborting every multi-chunk
            # upload. Give next_chunk() its own http with redirect-following DISABLED so it sees
            # the raw 308 and resumes normally. (Small metadata calls never emit a 308, so the
            # service's default http is fine everywhere else.)
            upload_http = httplib2.Http(timeout=300)
            upload_http.follow_redirects = False
            creds = service_account.Credentials.from_service_account_info(
                self.credentials_info, scopes=self.scopes
            )
            authed_http = AuthorizedHttp(creds, http=upload_http)
            response = None
            while response is None:
                status, response = request.next_chunk(http=authed_http, num_retries=5)
                if status:
                    print(f"   … {int(status.progress() * 100)}% uploaded", flush=True)
            uploaded = response

        self._share_anyone(service, uploaded.get('id'))
        print(f"🚀 Bag zip '{zip_name}' synced to Drive and shared via link.", flush=True)

    def _drive_file_size(self, service, file_id: str) -> int | None:
        """Returns a Drive file's byte size, or None if unavailable (Google-native files
        like Sheets report no size)."""
        try:
            meta = service.files().get(
                fileId=file_id, fields='size', supportsAllDrives=True
            ).execute()
            return int(meta['size']) if 'size' in meta else None
        except Exception:
            return None

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
        _lidar_metadata: dict[str, Any] | None = None
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
                    merged = {**_lidar_metadata, **flat_metrics}
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


    @classmethod
    def _flatten_metrics(cls, metrics: dict[str, Any]) -> dict[str, Any]:
        """Flatten the nested report ({zone}/{metric}/{sub}: value) into
        slash-joined column keys. Descends arbitrarily deep so it tracks the
        report structure; skips visualization blocks and per-cell dead-cell keys
        wherever they appear."""
        flat: dict[str, Any] = {}
        cls._flatten_into(metrics, '', flat)
        return flat

    @classmethod
    def _flatten_into(cls, node: Any, prefix: str, flat: dict[str, Any]) -> None:
        if not isinstance(node, dict):
            return
        for key, val in node.items():
            if key == 'visualization' or 'dead_cell_' in str(key) or 'worst_point_' in str(key):
                continue
            path = f'{prefix}/{key}' if prefix else str(key)
            if isinstance(val, dict):
                cls._flatten_into(val, path, flat)
            elif isinstance(val, (int, float, str)):
                flat[path] = val


    # ---- Reading (PolyView) ------------------------------------------------

    def _read_creds(self):
        """Service-account creds for the read APIs, built ONCE and reused. Sharing a single
        credentials object means the OAuth token is fetched once and reused across the
        thread-local Drive/Sheets clients — building fresh creds per client (as this did before)
        forced a separate key parse + token refresh per thread, which slowed every read."""
        creds = getattr(self, '_cached_read_creds', None)
        if creds is None:
            creds = service_account.Credentials.from_service_account_info(
                self.credentials_info, scopes=self._SCOPES)
            self._cached_read_creds = creds
        return creds

    @property
    def available(self) -> bool:
        return bool(self.credentials_info) and bool(self.root_folder_id)

    def clear_cache(self) -> None:
        with self._cache_lock:
            self._children_cache.clear()

    def retrieve_environments(self) -> list[str]:
        if not self.available:
            return []
        return list(self._list_children(self.root_folder_id, self._FOLDER_MIME).keys())

    def retrieve_env_data(self, env_name: str) -> dict[str, dict]:
        if not self.available:
            return {}
        env_id = self._list_children(self.root_folder_id, self._FOLDER_MIME).get(env_name)
        if env_id is None:
            return {}

        result, covered = self._read_env_summary(env_id, env_name)
        lidars = self._list_children(env_id, self._FOLDER_MIME)
        for lidar_name in lidars:
            result.setdefault(lidar_name, {})

        scan_lidars = {name: lid for name, lid in lidars.items() if name not in covered}
        if not scan_lidars:
            return result

        lidar_ids = list(scan_lidars.values())
        flat_by_lidar = self._list_children_by_parents(lidar_ids, self._SHEET_MIME)
        folders_by_lidar = self._list_children_by_parents(lidar_ids, self._FOLDER_MIME)

        jobs: list[tuple[str, str, str]] = []
        nested: list[tuple[str, str, str]] = []  # (lidar_name, case_path, case_folder_id)
        for lidar_name, lidar_id in scan_lidars.items():
            prefix = f'{lidar_name} - '
            for sheet_name, sheet_id in flat_by_lidar.get(lidar_id, {}).items():
                case_path = sheet_name[len(prefix):] if sheet_name.startswith(prefix) else sheet_name
                jobs.append((lidar_name, case_path, sheet_id))
            for case_path, case_id in folders_by_lidar.get(lidar_id, {}).items():
                nested.append((lidar_name, case_path, case_id))

        if nested:
            sheets_by_folder = self._list_children_by_parents(
                [case_id for _, _, case_id in nested], self._SHEET_MIME
            )
            for lidar_name, case_path, case_id in nested:
                sheet_id = sheets_by_folder.get(case_id, {}).get(f'{lidar_name} - {case_path}')
                if sheet_id is not None:
                    jobs.append((lidar_name, case_path, sheet_id))

        if not jobs:
            return result


        def _load(job: tuple[str, str, str]):
            _, _, sheet_id = job
            try:
                return job, self._read_case_metrics(sheet_id, propagate_errors=True)
            except (HttpError, OSError):
                return job, None 

        pending = jobs
        for attempt in range(self._READ_ATTEMPTS):
            failed: list[tuple[str, str, str]] = []
            with ThreadPoolExecutor(max_workers=self._MAX_WORKERS) as pool:
                for job, metrics in pool.map(_load, pending):
                    if metrics is None:
                        failed.append(job)
                        continue
                    if not metrics:
                        continue
                    lidar_name, case_path, _ = job
                    segments = case_path.split('/') if case_path else []
                    self._insert_nested(result[lidar_name], segments, metrics)
            if not failed:
                break
            pending = failed
            if attempt < self._READ_ATTEMPTS - 1:
                time.sleep(2 ** attempt)  
        if failed:
            print(f'[PolyView] WARNING: {len(failed)} case sheet(s) could not be read after retries.')
        return result

    def retrieve_visualization_data(self, env_name: str, lidar_name: str, case_path: str) -> dict:
        if not self.available:
            return {}
        env_id = self._list_children(self.root_folder_id, self._FOLDER_MIME).get(env_name)
        if env_id is None:
            return {}
        lidar_id = self._list_children(env_id, self._FOLDER_MIME).get(lidar_name)
        if lidar_id is None:
            return {}
        sheet_name = f'{lidar_name} - {case_path}'

        spreadsheet_id = None
        case_id = self._list_children(lidar_id, self._FOLDER_MIME).get(case_path)
        if case_id is not None:
            spreadsheet_id = self._list_children(case_id, self._SHEET_MIME).get(sheet_name)

        if spreadsheet_id is None:
            spreadsheet_id = self._list_children(lidar_id, self._SHEET_MIME).get(sheet_name)
        if spreadsheet_id is None:
            return {}
        return self._parse_viz_tab(spreadsheet_id)

    def retrieve_bag_download_link(self, env_name: str, lidar_name: str, case_path: str) -> str | None:
        """Returns a direct, click-to-download URL for the case's rosbag zip, or None if no zip
        is on Drive. The zip is shared 'anyone with link' at upload time, so this URL downloads
        without a login (large files show Google's scan-warning page once, then download).
        """
        if not self.available:
            return None
        env_id = self._list_children(self.root_folder_id, self._FOLDER_MIME).get(env_name)
        if env_id is None:
            return None
        lidar_id = self._list_children(env_id, self._FOLDER_MIME).get(lidar_name)
        if lidar_id is None:
            return None
        case_id = self._list_children(lidar_id, self._FOLDER_MIME).get(case_path)
        if case_id is None:
            return None
        for name, file_id in self._list_all_children(case_id).items():
            if name.lower().endswith('.zip'):
                return f'https://drive.google.com/uc?export=download&id={file_id}'
        return None

    def _drive_service(self):
        svc = getattr(self._thread_local, 'drive', None)
        if svc is None:
            svc = build('drive', 'v3', credentials=self._read_creds(), cache_discovery=False)
            self._thread_local.drive = svc
        return svc

    def _sheets_service(self):
        svc = getattr(self._thread_local, 'sheets', None)
        if svc is None:
            svc = build('sheets', 'v4', credentials=self._read_creds(), cache_discovery=False)
            self._thread_local.sheets = svc
        return svc

    def _list_children(self, parent_id: str, mime_type: str) -> dict[str, str]:
        cache_key = (parent_id, mime_type)
        with self._cache_lock:
            cached = self._children_cache.get(cache_key)
        if cached is not None:
            return cached
        children: dict[str, str] = {}
        page_token = None
        while True:
            response = self._drive_service().files().list(
                q=f"'{parent_id}' in parents and mimeType = '{mime_type}' and trashed = false",
                fields='nextPageToken, files(id, name)',
                supportsAllDrives=True,
                includeItemsFromAllDrives=True,
                corpora='allDrives',
                pageSize=100,
                pageToken=page_token,
            ).execute(num_retries=self._NUM_RETRIES)
            for item in response.get('files', []):
                children[item['name']] = item['id']
            page_token = response.get('nextPageToken')
            if not page_token:
                break
        with self._cache_lock:
            self._children_cache[cache_key] = children
        return children

    def _list_children_by_parents(self, parent_ids: list[str], mime_type: str) -> dict[str, dict[str, str]]:
        """List the children (of one mime type) of many parent folders in a few batched
        Drive queries instead of one round-trip per parent.

        OR's up to ``_PARENTS_PER_QUERY`` parent ids into a single ``files.list`` call and
        buckets the results by parent. Returns ``{parent_id: {name: id}}`` and seeds the
        per-parent ``_list_children`` cache so later single-parent lookups hit the cache.
        """
        by_parent: dict[str, dict[str, str]] = {pid: {} for pid in parent_ids}
        for start in range(0, len(parent_ids), self._PARENTS_PER_QUERY):
            chunk = parent_ids[start:start + self._PARENTS_PER_QUERY]
            parent_clause = ' or '.join(f"'{pid}' in parents" for pid in chunk)
            page_token = None
            while True:
                response = self._drive_service().files().list(
                    q=f"({parent_clause}) and mimeType = '{mime_type}' and trashed = false",
                    fields='nextPageToken, files(id, name, parents)',
                    supportsAllDrives=True,
                    includeItemsFromAllDrives=True,
                    corpora='allDrives',
                    pageSize=1000,
                    pageToken=page_token,
                ).execute(num_retries=self._NUM_RETRIES)
                for item in response.get('files', []):
                    for parent in item.get('parents', []):
                        if parent in by_parent:
                            by_parent[parent][item['name']] = item['id']
                page_token = response.get('nextPageToken')
                if not page_token:
                    break
        with self._cache_lock:
            for pid, names in by_parent.items():
                self._children_cache[(pid, mime_type)] = names
        return by_parent

    def _list_all_children(self, parent_id: str) -> dict[str, str]:
        """Like _list_children but without a mime-type filter — used to find the bag zip,
        whose stored mime type may vary (application/zip vs x-zip-compressed)."""
        cache_key = (parent_id, '*')
        with self._cache_lock:
            cached = self._children_cache.get(cache_key)
        if cached is not None:
            return cached
        children: dict[str, str] = {}
        page_token = None
        while True:
            response = self._drive_service().files().list(
                q=f"'{parent_id}' in parents and trashed = false",
                fields='nextPageToken, files(id, name)',
                supportsAllDrives=True,
                includeItemsFromAllDrives=True,
                corpora='allDrives',
                pageSize=100,
                pageToken=page_token,
            ).execute(num_retries=self._NUM_RETRIES)
            for item in response.get('files', []):
                children[item['name']] = item['id']
            page_token = response.get('nextPageToken')
            if not page_token:
                break
        with self._cache_lock:
            self._children_cache[cache_key] = children
        return children

    def _insert_nested(self, root: dict, segments: list[str], leaf: dict) -> None:
        if not segments:
            root.update(leaf)
            return
        cursor = root
        for seg in segments[:-1]:
            key = self._parse_segment_key(seg)
            cursor = cursor.setdefault(key, {})
        last_key = self._parse_segment_key(segments[-1])
        cursor[last_key] = leaf

    def _parse_segment_key(self, name: str) -> Any:
        if '=' in name:
            _, raw = name.split('=', 1)
            return self._cast_value(raw)
        if '_' in name:
            _, raw = name.rsplit('_', 1)
            val = self._cast_value(raw)
            if not isinstance(val, str):
                return val
        return name

    def _cast_value(self, raw: Any) -> Any:
        if not isinstance(raw, str):
            return raw
        if raw.lower() == 'true':
            return True
        if raw.lower() == 'false':
            return False
        try:
            return int(raw)
        except ValueError:
            pass
        try:
            return float(raw)
        except ValueError:
            pass
        return raw

    def _read_case_metrics(self, spreadsheet_id: str, propagate_errors: bool = False) -> dict:
        try:
            response = self._sheets_service().spreadsheets().values().get(
                spreadsheetId=spreadsheet_id,
                range='Sheet1!1:2',
                valueRenderOption='UNFORMATTED_VALUE',
            ).execute(num_retries=self._NUM_RETRIES)
        except (HttpError, OSError):
            # OSError covers socket/TimeoutError. A failed read is NOT an empty
            # case — when reading concurrently the caller retries these so a
            # throttled request doesn't silently drop a case from the overview.
            if propagate_errors:
                raise
            return {}
        rows = response.get('values', [])
        if len(rows) < 2:
            return {}
        return self._reshape_row(rows[0], rows[1])

    def _reshape_row(self, headers: list, values: list) -> dict:
        """Reshape one flat metrics row into the nested `{Metric: {zone_sub: value}}` form
        the app expects. Sheet columns are written zone-first by the reporter
        (`{zone}/{Metric}/{sub}`); identity/empty columns are skipped. Shared by the
        per-case reader and the environment-summary reader."""
        metrics: dict[str, dict] = {}
        for header, value in zip(headers, values):
            if header in ('', 'Case Name', 'Lidar') or value == '':
                continue
            casted = self._cast_value(value)
            parts = str(header).split('/')
            if len(parts) >= 3:
                zone, metric = parts[0], parts[1]
                sub = '_'.join(parts[2:])
                metrics.setdefault(metric, {})[f'{zone}_{sub}'] = casted
            elif len(parts) == 2:
                # Run-global scalars (e.g. `__global__/<sub>`) — keep as-is.
                top, sub = parts
                metrics.setdefault(top, {})[sub] = casted
            else:
                metrics.setdefault('lidar_metadata', {})[str(header)] = casted
        return metrics

    def _read_env_summary(self, env_id: str, env_name: str) -> tuple[dict[str, dict], set[str]]:
        """Read the single "<env> - summary" sheet (one row per case) in ONE call and rebuild
        the `{lidar: {case tree}}` structure.

        Returns ``(result, covered_lidars)`` where ``covered_lidars`` is the set of lidar
        names the summary actually supplied case data for. When no summary sheet exists (or it
        is empty/unreadable), returns ``({}, set())`` so the caller scans every lidar's per-case
        sheets. A lidar absent from ``covered_lidars`` is one the caller must scan itself."""
        summary_id = self._list_children(env_id, self._SHEET_MIME).get(f'{env_name} - summary')
        if summary_id is None:
            return {}, set()
        try:
            response = self._sheets_service().spreadsheets().values().get(
                spreadsheetId=summary_id,
                range='Sheet1',
                valueRenderOption='UNFORMATTED_VALUE',
            ).execute(num_retries=self._NUM_RETRIES)
        except (HttpError, OSError):
            return {}, set()
        rows = response.get('values', [])
        if len(rows) < 2:
            return {}, set()
        header = rows[0]
        result: dict[str, dict] = {}
        covered: set[str] = set()
        for row in rows[1:]:
            if len(row) < 2:
                continue
            lidar_name, case_path = str(row[0]), str(row[1])
            metrics = self._reshape_row(header[2:], row[2:])
            if not metrics:
                continue
            result.setdefault(lidar_name, {})
            covered.add(lidar_name)
            segments = case_path.split('/') if case_path else []
            self._insert_nested(result[lidar_name], segments, metrics)
        return result, covered

    def _parse_viz_tab(self, spreadsheet_id: str) -> dict:
        result: dict = {
            'profile_plane': {},
            'orientation': {},
            'fitted_planes': {},
            'dead_cells': {},
            'worst_points': {},
            'roi_cloud': None,
            'filtered_roi_cloud': None,
        }
        try:
            response = self._sheets_service().spreadsheets().values().get(
                spreadsheetId=spreadsheet_id,
                range='Visualization',
            ).execute(num_retries=self._NUM_RETRIES)
        except (HttpError, OSError):
            return result
        rows = response.get('values', [])

        current_section: str | None = None
        is_cloud: bool = False
        current_cloud_key: str | None = None
        cloud_chunks: list[str] = []

        def _flush_cloud():
            if is_cloud and cloud_chunks and current_cloud_key:
                try:
                    result[current_cloud_key] = self._decode_cloud(''.join(cloud_chunks))
                except Exception as e:
                    print(f'[PolyView] ERROR decoding cloud "{current_cloud_key}": {e}')
                cloud_chunks.clear()

        for row in rows:
            if not row:
                continue
            text = str(row[0])
            if not text:
                continue

            # Bullet "key: value" line — only when we're in a non-cloud section
            if ': ' in text and current_section is not None and not is_cloud:
                key, raw = text.split(': ', 1)
                result[current_section][key] = self._cast_value(raw)
                continue

            # Section heading detection — these strings never appear inside base64 chunks
            # because the writer separates header text with the U+00B7 mid-dot, which is
            # outside the base64 alphabet.
            is_heading = (
                text in ('Orientation', 'ProfilePlane', 'WorstPoints')
                or ' · FittedPlane' in text
                or ' · DeadCells' in text
                or 'base64' in text
            )

            if is_heading:
                _flush_cloud()
                if 'base64' in text:
                    current_cloud_key = text.split(' ·')[0].strip()
                    is_cloud = True
                    current_section = None
                else:
                    is_cloud = False
                    current_cloud_key = None
                    if text == 'ProfilePlane':
                        current_section = 'profile_plane'
                    elif text == 'Orientation':
                        current_section = 'orientation'
                    elif 'FittedPlane' in text:
                        current_section = 'fitted_planes'
                    elif 'DeadCells' in text:
                        current_section = 'dead_cells'
                    elif text == 'WorstPoints':
                        current_section = 'worst_points'
                    else:
                        current_section = None
                continue

            # Anything left, while we're inside a cloud block, is a base64 chunk
            if is_cloud:
                cloud_chunks.append(text)

        _flush_cloud()
        return result

    def _decode_cloud(self, encoded: str) -> np.ndarray:
        cleaned = encoded.strip()
        cleaned += '=' * (-len(cleaned) % 4)
        raw = np.frombuffer(base64.b64decode(cleaned), dtype=np.float32)
        n = len(raw)
        if n % 4 == 0:
            return raw.reshape(-1, 4)
        if n % 3 == 0:
            xyz = raw.reshape(-1, 3)
            return np.hstack([xyz, np.zeros((len(xyz), 1), dtype=np.float32)])
        raise ValueError(f'Point cloud buffer has {n} floats, not divisible by 3 or 4')
