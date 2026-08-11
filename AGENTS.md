# Agent Guidelines

## Priorities

Prioritize correctness, simplicity, consistency, robustness, and
maintainability, in that order. Do not add complexity for hypothetical needs.

Keep changes within the requested scope. Inspect existing code and follow an
established pattern before creating a new one. Treat `prototypes/` as reference
material, not as the production architecture.

## Code

Follow the Google style guide for each supported language. Keep lines at or
below 80 characters unless a tool or project convention requires otherwise.
Use explicit control flow and braces in C, C++, and Rust.

Comments must explain current constraints or non-obvious decisions. Do not add
comments that describe removed code, failed attempts, or temporary debugging.

For C, C++, Rust, and firmware, check bounds, integer conversions, resource
ownership, and plausible failure paths. Avoid unjustified dynamic allocation
in firmware.

## Hardware Safety

Do not assume wiring, pin assignments, bus addresses, voltage levels, hardware
revisions, or calibration values. Verify them from provided evidence.

Do not flash hardware, erase memory, change fuses or calibration, actuate
outputs, or transmit on a live bus without explicit permission.

## Verification

Run relevant formatting, linting, static analysis, and build checks. Do not add
test infrastructure unless requested. Never report a check as passing unless
it was run and observed.

## Git and Documentation

All commit messages must follow Conventional Commits. Agents may create commits
when the user authorizes them. Do not mention Claude, Codex, or another AI tool
in a commit message, and do not add one as a co-author.

Do not rewrite history, force-push, or discard user changes without explicit
permission.

Keep documentation concise. Do not present undecided architecture as fact.
