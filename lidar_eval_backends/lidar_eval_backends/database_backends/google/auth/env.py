# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

import os
from pathlib import Path

from lidar_eval_backends.authentication_interface import AuthInterface


class Env(AuthInterface):
    """Load the Google backend's credentials from an `auth.env` file — a plain KEY=VALUE list of
    the service-account fields plus `root_folder_id`. Non-interactive and dependency-free: the
    generic (non-Polymath) alternative to the 1Password provider.

    The file's location comes from `env_file` in this provider's auth_registry.yaml row, with
    `$AUTH_ENV_FILE` as an override. `~` and `$VARS` in the configured value are expanded.

    This provider deliberately does no path derivation of its own. auth.env holds real credentials,
    so it is gitignored and never installed — meaning there is no package resource to resolve it
    through (the way the registry YAMLs themselves are resolved), and counting parent directories
    from this module gives a different answer depending on whether the package is running from
    source or from `install/`. Declaring the path in the registry keeps that knowledge in
    configuration instead of in library code.
    """

    def _configured_path(self) -> Path:
        # An override never falls back to the registry value: a typo in AUTH_ENV_FILE should fail
        # loudly rather than quietly authenticating as whoever the configured file belongs to.
        raw = os.environ.get('AUTH_ENV_FILE') or self.config.get('env_file')
        if not raw:
            raise ValueError(
                'No auth.env location configured. Set `config.env_file` on the Env row in '
                'google/auth_registry.yaml, or set $AUTH_ENV_FILE.'
            )
        return Path(os.path.expandvars(str(raw))).expanduser()

    def authenticate(self) -> dict:
        path = self._configured_path()
        if not path.is_file():
            raise FileNotFoundError(
                f'auth.env not found at {path}. Copy auth.env.example there and fill it in, or '
                'point `config.env_file` in google/auth_registry.yaml at the right location.'
            )
        blob: dict = {}
        for raw in path.read_text().splitlines():
            line = raw.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            key, _, value = line.partition('=')
            value = value.strip().strip('"').strip("'")
            blob[key.strip()] = value.replace('\\n', '\n')   # unescape newlines (private_key)
        if not blob:
            raise ValueError(f'auth.env at {path} has no KEY=VALUE entries.')
        return blob
