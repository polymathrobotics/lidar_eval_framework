from abc import ABC, abstractmethod


class AuthInterface(ABC):
    """Contract for a credential provider: fetch a backend's credentials from wherever they live
    (1Password, env vars, a key file, a cloud metadata service) and return them as one opaque blob
    for that backend's load_credentials().

    Providers are scoped to the backend whose blob shape they produce (e.g. Google's providers live
    under database_backends/google/auth/), so no cross-backend key negotiation is needed — each
    provider statically knows the shape it must return.

    Each provider is constructed with the `config` block from its own row in the backend's
    auth_registry.yaml, so anything deployment-specific a provider needs (a file path, an item
    reference, a vault name) is declared alongside the choice of provider rather than derived
    inside the provider. Providers that need no configuration simply ignore it.
    """

    def __init__(self, config: dict | None = None):
        self.config: dict = config or {}

    @abstractmethod
    def authenticate(self) -> dict:
        """Fetch credentials and return them as an opaque blob for a backend's load_credentials()."""
        raise NotImplementedError
