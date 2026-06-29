---
sidebar_position: 4
title: Contribute
---

# 🤝 Contribute

We welcome pull requests! To keep the deployment and integration pipelines clean and stable, please adhere to our shared development lifecycle rules.

## Opening a Pull Request (PR)
1. Isolate your work by creating a feature branch off of `main`: `feat/add-sensor-x` or `fix/matrix-transform`.
2. Format your commit descriptions using standard **Conventional Commits** formatting keys (e.g., `feat(core): add type-safe transform matrix`).

## Writing & Running Verification Tests
Never merge changes without corresponding test code. Place your target test suites directly inside the project boundary `__tests__/` directory.

```bash
# Execute the absolute test suite suite across workspaces
npm run test
