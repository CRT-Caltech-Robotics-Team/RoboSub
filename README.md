# RoboSub

Code base for the Caltech Robotics Team (CRT) RoboSub 2026-2027 season.

Documentation: <https://crt.robosub.io/>

## Repository Layout

```text
.
├── .github/workflows/  GitHub Actions checks
├── docs/               MkDocs documentation
├── prototypes/eddie/   Earlier prototype code
├── AGENTS.md           Coding-agent guidelines
├── commitlint.config.mjs  Commit-message rules
├── mkdocs.yml          Documentation configuration
├── package.json        Repository tooling
└── README.md           Project overview
```
## Documentation

Install and serve the documentation locally:

```sh
python3 -m pip install -r docs/requirements.txt
python3 -m mkdocs serve
```
## Commits
Use a short Conventional Commit subject:

```text
feat: add camera capture
fix(control): clamp thrust output
docs: update setup instructions
```

Scopes are optional. Pull requests check commit subjects before merge.
