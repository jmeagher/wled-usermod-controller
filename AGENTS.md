# AGENTS.md

Guidance for AI coding agents and contributors. See [README.md](README.md) for what this project is and how to build it; tests are `tests/run.sh` and `python3 -I -m unittest discover -s tests -v`.

## Publishing hygiene (this repository is public)

Never commit details of a private network or a secret, in code, docs, tests, examples, fixtures, commit messages or PR text:

- **IP addresses:** use the documentation range `192.0.2.0/24` (RFC 5737) or an mDNS name such as `wled-1234.local`. Never a real LAN address (10/8, 172.16/12, 192.168/16). WLED's factory AP address `192.168.4.1` is the one allowed exception.
- **MAC addresses:** when pasting tool output into a test, replace the MAC with a made-up locally-administered one such as `02:00:00:00:00:01`. A real MAC identifies a physical device.
- **Credentials:** no Wi-Fi/AP/OTA passwords, tokens or keys. Real values live only in the git-ignored `.wled.secrets`; tests use obvious fakes.
- **Paths and captures:** no `/Users/<name>` or `/home/<name>` paths, no raw `/json/cfg`, `presets.json` or serial/esptool dumps without scrubbing, and no flash images, backups or logs.
- **Device and room names are fine** (they are documented examples), but do not add a street address, a person's full name or similar.

`tests/test_leaks.py` enforces the mechanical part of this on every tracked file and runs in CI. A deliberate, harmless exception can carry the marker `leak-ok` on its line. History is not rewritten, so anything committed stays public: scrub before you commit, not after.
