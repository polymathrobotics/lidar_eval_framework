---
sidebar_position: 3
title: Developer Guide
---

# 🛠️ Developer Guide

If you are expanding functionality, adding custom evaluation criteria, or tuning file parsing performance limits, keep your changes unified by adhering to our local guidelines.

## Local Workflows
Avoid modifying operational logic directly inside active container mounts. Use your local editor environment and run verification scripts at regular intervals:
* **Linting:** Run `npm run lint` to enforce unified formatting rules.
* **Static Compilations:** Run `npm run typecheck` to keep code structures completely type-safe.
