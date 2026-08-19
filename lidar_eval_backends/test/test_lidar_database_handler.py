# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

"""
High-level tests for LidarDatabaseHandler — backend selection and delegation.

No real backend is contacted: these cover which backend the registry selects and
how the handler behaves with and without one.
"""

from lidar_eval_backends.lidar_database_handler import LidarDatabaseHandler
import pytest
import yaml


class StubBackend:
    """Records what the handler delegates to it."""

    def __init__(self, available=True, auth_error=None):
        self._available = available
        self._auth_error = auth_error
        self.authenticate_calls = 0
        self.synced = None
        self.pushed = None
        self.loaded_credentials = None

    @property
    def available(self):
        return self._available

    def authenticate(self):
        self.authenticate_calls += 1
        if self._auth_error is not None:
            raise self._auth_error

    def load_credentials(self, credentials):
        self.loaded_credentials = credentials

    def sync(self, test_data, rosbags, lidar_metadata):
        self.synced = (test_data, rosbags, lidar_metadata)

    def push_visualization_to_case(self, env, lidar, case, blocks):
        self.pushed = (env, lidar, case, blocks)


def registry(tmp_path, enabled):
    """Write a database_registry.yaml selecting (or disabling) the Google backend."""
    path = tmp_path / 'database_registry.yaml'
    path.write_text(yaml.safe_dump({'database_registry': [{
        'database_backend': 'Google',
        'executable': 'google.google_services_handler',
        'class': 'GoogleServicesHandler',
        'enabled': enabled,
    }]}))
    return path


def test_shipped_registry_selects_the_google_backend():
    handler = LidarDatabaseHandler()

    assert type(handler._database_handler).__name__ == 'GoogleServicesHandler'


def test_a_registry_with_nothing_enabled_configures_no_backend(tmp_path):
    handler = LidarDatabaseHandler(registry_path=registry(tmp_path, enabled=False))

    assert handler._database_handler is None
    assert handler.available is False


def test_reads_degrade_to_empty_defaults_without_a_backend(tmp_path):
    handler = LidarDatabaseHandler(registry_path=registry(tmp_path, enabled=False))

    assert handler.retrieve_environments() == []
    assert handler.retrieve_env_data('E1') == {}
    assert handler.retrieve_visualization_data('E1', 'at128', 'base') == {}
    assert handler.retrieve_bag_download_link('E1', 'at128', 'base') is None
    handler.clear_cache()   # no-op rather than an error


def test_authenticate_without_a_backend_raises(tmp_path):
    handler = LidarDatabaseHandler(registry_path=registry(tmp_path, enabled=False))

    with pytest.raises(RuntimeError, match='No database backend configured'):
        handler.authenticate()
    assert handler.authenticated is False


def test_sync_authenticates_once_then_delegates(tmp_path):
    handler = LidarDatabaseHandler(
        lidar_metadata={'model': 'AT128'}, registry_path=registry(tmp_path, enabled=False),
    )
    backend = StubBackend()
    handler._database_handler = backend
    tree = {'E1': {'at128': {'base': {'wall': {'M': {'sub': 1.0}}}}}}

    handler.sync(tree, rosbags={'at128': {'base': ['/bags/1']}})
    handler.push_visualization('E1', 'at128', 'base', [['roi_cloud']])

    assert backend.authenticate_calls == 1          # cached across both calls
    assert backend.synced == (tree, {'at128': {'base': ['/bags/1']}}, {'model': 'AT128'})
    assert backend.pushed == ('E1', 'at128', 'base', [['roi_cloud']])


def test_sync_does_not_raise_when_authentication_fails(tmp_path):
    handler = LidarDatabaseHandler(registry_path=registry(tmp_path, enabled=False))
    handler._database_handler = StubBackend(available=False, auth_error=RuntimeError('no creds'))

    handler.sync({})   # a missing secret must not take the reporting node down

    assert handler.authenticated is False


def test_credentials_are_passed_straight_through_to_the_backend(tmp_path):
    handler = LidarDatabaseHandler(registry_path=registry(tmp_path, enabled=False))
    backend = StubBackend()
    handler._database_handler = backend

    handler.load_credentials({'root_folder_id': 'abc123'})

    assert backend.loaded_credentials == {'root_folder_id': 'abc123'}
